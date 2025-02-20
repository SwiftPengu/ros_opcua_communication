#!/usr/bin/env python

# Thanks to:
# https://github.com/ros-visualization/rqt_common_plugins/blob/groovy-devel/rqt_topic/src/rqt_topic/topic_widget.py
import numpy
import random

import roslib
import roslib.message
import rospy
from opcua import ua, uamethod
import opcua

import ros_actions
import rostopic
import refreshing
import common
import itertools

import traceback


#debug
import sys

class OpcUaROSTopic:
    def __init__(self, server, parent, idx, topic_name, topic_type):
        self.server = server
        self.constructed = False
        self.idx = idx
        self.parent = self.recursive_create_objects(topic_name, parent)
        self.type_name = topic_type
        self.name = topic_name
        self._nodes = {}

        self.message_class = None
        try:
            rospy.logdebug("Creating ROS Topic with name: {} and type: {}...".format(self.name, self.type_name))
            self.message_class = roslib.message.get_message_class(topic_type)
            self.message_instance = self.message_class()

            self._recursive_create_items(self.parent, topic_name, topic_type, self.message_instance, True)

            self._subscriber = rospy.Subscriber(self.name, self.message_class, self.message_callback)
            self._publisher = rospy.Publisher(self.name, self.message_class, queue_size=1)
            rospy.loginfo("Created ROS Topic with name: {} and type: {}!".format(self.name, self.type_name))
        except (rospy.ROSException, TypeError) as e:
            self.message_class = None
            rospy.logfatal("Couldn't find message class for type " + topic_type)

            raise e
        self.constructed = True
        # if self.name == '/rosout':
            # quit()
            # pass


    def _recursive_create_items(self, parent, topic_name, type_name, message, top_level=False):
        rospy.logdebug('Creating item for {} | {}'.format(topic_name, type_name))
        topic_text = node_name = topic_name.split('/')[-1]
        if '[' in topic_text:
            topic_text = topic_text[topic_text.index('['):]

        # skip dynamic type definitions
        if 'dynamic_reconfigure' in type_name:
            return

        # This here are 'complex datatypes'
        if hasattr(message, '__slots__') and hasattr(message, '_slot_types'):
            rospy.logdebug('Creating struct {}'.format(topic_name))
            # rospy.logdebug('{} / {}'.format(message.__slots__, message._slot_types))
            
            # Create node and add a type property
            newNodeId = ua.NodeId(topic_name, parent.nodeid.NamespaceIndex, ua.NodeIdType.String)
            
            existingNodes = filter(lambda child: child.nodeid.Identifier == newNodeId.Identifier, parent.get_children())
            new_node = None
            if len(existingNodes) == 0:
                # map(lambda child: child.get_node, parent.get_children())
                new_node = parent.add_object(newNodeId, ua.QualifiedName(node_name, parent.nodeid.NamespaceIndex))
                new_node.add_property(ua.NodeId(topic_name + ".Type", self.idx),
                                    ua.QualifiedName("Type", parent.nodeid.NamespaceIndex), type_name)
        

                # Add an 'update' method, which ??? (publish to ROS) ?
                if top_level:
                    new_node.add_method(ua.NodeId(topic_name + ".Update", parent.nodeid.NamespaceIndex),
                                        ua.QualifiedName("Update", parent.nodeid.NamespaceIndex),
                                        self.opcua_update_callback, [], [])
            else:
                new_node = existingNodes[0]
                
            # Process child nodes
            for slot_name, type_name_child in zip(message.__slots__, message._slot_types):
                rospy.logdebug('Creating child for {}, slot_name: {}, slot_type: {}'.format(topic_name, slot_name, type_name_child))
                self._recursive_create_items(
                        parent=new_node,
                        topic_name='{}/{}'.format(topic_name, slot_name),
                        type_name=type_name_child,
                        message=getattr(message, slot_name)
                    )
                
            # Store node in the index
            self._nodes[topic_name] = new_node

        elif type(message) in (list, tuple):
            rospy.logdebug('Creating array item for {} ({})'.format(topic_name, type_name))
            # These are arrays
            base_type_str, array_size = _extract_array_info(type_name)
            try:
                base_instance = roslib.message.get_message_class(base_type_str)()
            except (ValueError, TypeError):
                base_instance = None
                
            rospy.logdebug('Base array type is: {}[{}]'.format(base_type_str, array_size))


            if array_size is not None and hasattr(base_instance, '__slots__'):
                # Create nodes for bounded arrays
                # TODO check if parent item is required
                for index in range(array_size):
                    self._recursive_create_items(parent, topic_name + '[%d]' % index, base_type_str, base_instance)
            else:
                rospy.logdebug('Creating typed array node')
                # rospy.logdebug('Creating item for unbounded arrays')
                new_node = parent.add_object(ua.NodeId(topic_name, parent.nodeid.NamespaceIndex, ua.NodeIdType.String),
                                        ua.QualifiedName(node_name, parent.nodeid.NamespaceIndex))
                new_node.add_property(ua.NodeId(topic_name + ".Type", self.idx),
                                ua.QualifiedName("Type", parent.nodeid.NamespaceIndex), type_name)
                self._nodes[topic_name] = new_node
        else:
            rospy.logdebug('Creating variable')
            new_node = _create_node_with_type(parent, topic_name, topic_text, type_name)
            if new_node is not None:
                self._nodes[topic_name] = new_node
            else:
                rospy.logwarn('Error creating node with type topic: {}, type: {}, skipping creation'.format(topic_name, type_name))

        if topic_name in self._nodes and self._nodes[topic_name].get_node_class() == ua.NodeClass.Variable:
            self._nodes[topic_name].set_writable(True)
        return

    def message_callback(self, message):
        # rospy.loginfo('--- message cb | {}'.format(self.name))
        # Only update if type/child construction has finished
        if self.constructed:
            self.update_value(self.name, message)
            sys.stdout.flush()

    # Method to force request a value update towards ROS
    @uamethod
    def opcua_update_callback(self, parent):
        try:
            for nodeName in self._nodes:
                child = self._nodes[nodeName] # look up value for the key
                childName = child.get_display_name().Text # get child name
                if hasattr(self.message_instance, childName):
                    if child.get_node_class() == ua.NodeClass.Variable:
                        setattr(self.message_instance, childName,
                                correct_type(child, type(getattr(self.message_instance, childName))))
                    elif child.get_node_class == ua.NodeClass.Object:
                        setattr(self.message_instance, childName, self.create_message_instance(child))
            # Publish OPC-UA data to ROS ??
            self._publisher.publish(self.message_instance)
        except rospy.ROSException as e:
            rospy.logerr("Error when updating node " + self.name, e)
            self.server.server.delete_nodes([self.parent]) # FIXME this may cause even more problems?

    # Method which publishes ROS topic updates to OPC-UA
    def update_value(self, topic_name, message):
        rospy.logdebug('Updating: {}'.format(topic_name))
        
        # skip dynamic reconfigure
        if 'parameter_descriptions' in topic_name:
            return
        if 'parameter_updates' in topic_name:
            return

        # Structs
        if hasattr(message, '__slots__') and hasattr(message, '_slot_types'):
            self._update_struct(topic_name, message)

        # Lists
        elif type(message) in (list, tuple):
            # Some messages have slots (e.g. multipart messages)
            self._update_list(topic_name, message)

            # remove obsolete children
            if topic_name in self._nodes:
                # more robust child count
                childCount = len(filter(lambda c: c.get_display_name().Text != 'Type', self._nodes[topic_name].get_children()))
                if len(message) < childCount:
                    for i in range(len(message), childCount):
                        # FIXME broken with new array support feature
                        # rospy.loginfo('Warning, deleting stuff!')
                        # item_topic_name = topic_name + '[%d]' % i
                        # self.recursive_delete_items(self._nodes[item_topic_name])
                        # del self._nodes[item_topic_name]
                        pass
        else:
            self._update_val(topic_name, message)

    def _update_struct(self, topic_name, message):
        rospy.logdebug('Update struct')
        for slot_name in message.__slots__:
            self.update_value('{}/{}'.format(topic_name,slot_name), getattr(message, slot_name))

    def _update_list(self, topic_name, message):
        rospy.logdebug('Update list')
        if (len(message) > 0):
            # rospy.loginfo('[{}] Processing array, len: {}'.format(topic_name, len(message)))
            # enumerate(message)
            if topic_name in self._nodes:
                # Check if the topic name is known (this is expected, the else case is probably a bug)
                # Get the type property
                typeProp = None
                try:
                    typeProp = next(itertools.ifilter(lambda p: p.get_display_name().Text == 'Type', self._nodes[topic_name].get_properties()))
                except StopIteration:
                    rospy.logerr('Bug? Node has no type! Topic: {}, Message: {}'.format(topic_name, message))
                    rospy.loginfo(self._nodes[topic_name])
                    return

                # Remove array marks from the type to get the 'base_type'
                type_str = typeProp.get_value()
                assert(type_str[-2:] == '[]')
                base_type_str = typeProp.get_value()[:-2]

                # Iterate over all array elements
                for index, slot in enumerate(message):
                    indexNodeName = '{}[{}]'.format(topic_name, index)
                    # Check if array element exists, and update that
                    if indexNodeName in self._nodes:
                        self.update_value(indexNodeName, slot)
                    else:
                        self._recursive_create_items(self._nodes[topic_name], indexNodeName, base_type_str, slot, None)
            else:
                rospy.logwarn('Node not handled, update for non-existing topic? ({})'.format(topic_name))
                    
                # TODO remove items which are missing (e.g. when a list moves from len 6 to len 4)
        else:
            rospy.logdebug('[{}] Message skipped (list), len: {} | {}'.format(topic_name, len(message), message))
            # rospy.loginfo('Message: {}'.format(str(message)))

    def _update_val(self, topic_name, message):
        rospy.logdebug('Message set to repr {}, repr: {}'.format(topic_name, repr(message)))
        if topic_name in self._nodes and self._nodes[topic_name] is not None:
            obj = self._nodes[topic_name]
            rospy.logdebug(obj)
            obj.set_value(repr(message))
        else:
            rospy.logwarn('Message skipped (repr), topic is unknown: {}, len: {}'.format(topic_name, len(message)))

    def recursive_delete_items(self, item):
        self._publisher.unregister()
        self._subscriber.unregister()
        for child in item.get_children():
            self.recursive_delete_items(child)
            if child in self._nodes:
                del self._nodes[child]
            self.server.server.delete_nodes([child])
        self.server.server.delete_nodes([item])
        if len(self.parent.get_children()) == 0:
            self.server.server.delete_nodes([self.parent])

    # Converts the value of the node to that specified in the ros message we are trying to fill.
    def create_message_instance(self, node):
        for child in node.get_children():
            name = child.get_display_name().Text
            if hasattr(self.message_instance, name):
                if child.get_node_class() == ua.NodeClass.Variable:
                    setattr(self.message_instance, name,
                            correct_type(child, type(getattr(self.message_instance, name))))
                elif child.get_node_class == ua.NodeClass.Object:
                    setattr(self.message_instance, name, self.create_message_instance(child))
        return self.message_instance

    def recursive_create_objects(self, topic_name, parent):
        hierachy = topic_name.split('/')
        if len(hierachy) == 0 or len(hierachy) == 1:
            return parent
        for name in hierachy:
            if name != '': # skip empty names (e.g. the first slash)
                try:
                    nodewithsamename = self.server.find_topics_node_with_same_name(name, self.idx)
                    if nodewithsamename is not None:
                        return self.recursive_create_objects(common.nextname(hierachy, hierachy.index(name)),
                                                             nodewithsamename)
                    else:
                        # if for some reason 2 services with exactly same name are created use hack>: add random int, prob to hit two
                        # same ints 1/10000, should be sufficient
                        newparent = parent.add_object(
                            ua.NodeId(name, parent.nodeid.NamespaceIndex, ua.NodeIdType.String),
                            ua.QualifiedName(name, parent.nodeid.NamespaceIndex))
                        return self.recursive_create_objects(common.nextname(hierachy, hierachy.index(name)),
                                                             newparent)
                # thrown when node with parent name is not existent in server or when nodeid already exists (perhaps that error should be unrecoverable)
                except (IndexError, opcua.ua.uaerrors._auto.BadNodeIdExists):
                    newparent = parent.add_object(
                        ua.NodeId(name + str(random.randint(0, 10000)), parent.nodeid.NamespaceIndex,
                                  ua.NodeIdType.String),
                        ua.QualifiedName(name, parent.nodeid.NamespaceIndex))
                    return self.recursive_create_objects(common.nextname(hierachy, hierachy.index(name)),
                                                         newparent)

        return parent


# to unsigned integers as to fulfill ros specification. Currently only uses a few different types,
# no other types encountered so far.
def correct_type(node, typemessage):
    data_value = node.get_data_value()
    result = node.get_value()
    if isinstance(data_value, ua.DataValue):
        if typemessage.__name__ == "float":
            result = numpy.float(result)
        if typemessage.__name__ == "double":
            result = numpy.double(result)
        if typemessage.__name__ == "int":
            result = int(result) & 0xff
    else:
        rospy.logerr("can't convert: " + str(node.get_data_value.Value))
        return None
    return result

# Obtains the array type and its size (if it has a static size)
def _extract_array_info(type_str):
    array_size = None
    if '[' in type_str and type_str[-1] == ']':
        type_str, array_size_str = type_str.split('[', 1)
        array_size_str = array_size_str[:-1]
        if len(array_size_str) > 0:
            array_size = int(array_size_str)

    return type_str, array_size

def _get_ua_variant(type_name, topic_name):
    if type_name == 'bool':
        dv = ua.Variant(False, ua.VariantType.Boolean)
    elif type_name == 'byte':
        dv = ua.Variant(0, ua.VariantType.Byte)
    elif type_name == 'int':
        dv = ua.Variant(0, ua.VariantType.Int32)
    elif type_name == 'int8':
        dv = ua.Variant(0, ua.VariantType.SByte)
    elif type_name == 'uint8':
        dv = ua.Variant(0, ua.VariantType.Byte)
    elif type_name == 'int16':
        dv = ua.Variant(0, ua.VariantType.Int16)
    elif type_name == 'uint16':
        dv = ua.Variant(0, ua.VariantType.UInt16)
    elif type_name == 'int32':
        dv = ua.Variant(0, ua.VariantType.Int32)
    elif type_name == 'uint32':
        dv = ua.Variant(0, ua.VariantType.UInt32)
    elif type_name == 'int64':
        dv = ua.Variant(0, ua.VariantType.Int64)
    elif type_name == 'uint64':
        dv = ua.Variant(0, ua.VariantType.UInt64)
    elif type_name == 'float' or type_name == 'float32' or type_name == 'float64':
        dv = ua.Variant(0.0, ua.VariantType.Float)
    elif type_name == 'double':
        dv = ua.Variant(0.0, ua.VariantType.Double)
    elif type_name == 'string':
        dv = ua.Variant('', ua.VariantType.String)
    else:
        raise TypeError('Unknown type {} for topic {}'.format(type_name, topic_name))
    
    return dv

def _create_node_with_type(parent, topic_name, topic_text, type_name):
    # Strip off array designations
    if '[' in type_name:
        type_name = type_name[:type_name.index('[')]
        
    try:
        variant = _get_ua_variant(type_name, topic_name)
    except TypeError:
        rospy.logwarn("can't create node with type {} for topic {}".format(type_name, topic_name))
        return None

    newNodeId = ua.NodeId(topic_name, parent.nodeid.NamespaceIndex)
    existingVariables = filter(lambda variable: variable.nodeid.Identifier == newNodeId.Identifier, parent.get_variables())
    newVariable = None
    if len(existingVariables) == 0:
        newVariable = parent.add_variable(newNodeId,
                                ua.QualifiedName(topic_text, parent.nodeid.NamespaceIndex), variant.Value)
    else:
        newVariable = existingVariables[0]
    return newVariable
    


# Used to delete obsolete topics
def numberofsubscribers(nametolookfor, topicsDict):
    if nametolookfor == "/rosout":
        return 2 # rosout only has one subscriber/publisher at all times
    return topicsDict[nametolookfor]._subscriber.get_num_connections()


def get_feedback_type(action_name):
    try:
        type, name, fn = rostopic.get_topic_type("{}/feedback".format(action_name))
        return type
    except rospy.ROSException as e:
        try:
            type, name, fn = rostopic.get_topic_type("{action_name}/Feedback", e)
            return type
        except rospy.ROSException as e2:
            rospy.logerr("Couldnt find feedback type for action {}".format(action_name), e2)
            return None


def get_goal_type(action_name):
    try:
        type, name, fn = rostopic.get_topic_type("{}/goal".format(action_name))
        return type
    except rospy.ROSException as e:
        try:
            type, name, fn = rostopic.get_topic_type("{}/Goal".format(action_name), e)
            return type
        except rospy.ROSException as e2:
            rospy.logerr("Couldnt find feedback type for action {}".format(action_name), e2)
            return None

def refresh_topics_and_actions(namespace_ros, topicsFilter, server, topicsdict, actionsdict, idx_topics, idx_actions, topics,
                               actions):
    ros_topics = rospy.get_published_topics(namespace_ros)
    # Apply topics filter
    ros_topics = [(topic_name, topic_type) for topic_name, topic_type in ros_topics  if topicsFilter(topic_name)]
    for topic_name, topic_type in ros_topics:
        # Create new topics if they are not in the current dict
        if topic_name not in topicsdict or topicsdict[topic_name] is None:
            if "cancel" in topic_name or "result" in topic_name or "feedback" in topic_name or "goal" in topic_name or "status" in topic_name:
                rospy.logdebug("Found an action: " + str(topic_name))
                
                correct_name = ros_actions.OpcUaROSAction.get_correct_name(topic_name)
                rospy.logdebug('Skipping generation of action {}'.format(correct_name))
            
            # FIXME crashes when message type is not known
            topic = OpcUaROSTopic(server, topics, idx_topics, topic_name, topic_type)
            assert(topic is not None)
            topicsdict[topic_name] = topic
        # if the topic was already seen, check if the topic has become ?? obsolete ??
        # elif numberofsubscribers(topic_name, topicsdict) <= 1 and "rosout" not in topic_name:
        #     topicsdict[topic_name].recursive_delete_items(server.server.get_node(ua.NodeId(topic_name, idx_topics)))
        #     del topicsdict[topic_name]

    # ros_topics = rospy.get_published_topics(namespace_ros)
    # All current topics which are also present at ROS, we have created all new topics, so only need to remove old topics.
    newTopics = {newTopicName: topicsdict[newTopicName] for newTopicName in topicsdict.keys() if newTopicName in [topic_name for topic_name, _ in ros_topics]}
    for topicName in topicsdict.keys(): # Check if any old services no longer exist
        if topicName not in newTopics:
            topic = topicsdict[topicName]

            # Delete all children of pruned nodes
            topic.recursive_delete_items(
                    server.server.get_node(
                        ua.NodeId(topicName, idx_topics)
                    )
                )
    
    # TODO move this function to ros_server, and directly assign dict
    # topicsdict.clear()
    topicsdict.update(newTopics)

    # refreshing.refresh_dict(namespace_ros, actionsdict, topicsdict, server, idx_actions)
