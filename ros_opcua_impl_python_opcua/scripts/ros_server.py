#!/usr/bin/python2
import sys
import time

import rosgraph
import rosnode
import rospy
import opcua

import ros_services
import ros_topics
import os

HOST = os.environ.get('HOST', '0.0.0.0')
PORT = os.environ.get('PORT', 21554)
NS = os.environ.get('NS', '/') # alternative for using the params.yaml file

# Returns the hierachy as one string from the first remaining part on.
def nextname(hierachy, last_processed_index):
    try:
        result = "".join(map(str, hierachy[last_processed_index+1:]))
    except Exception as e:
        rospy.logerr("Error encountered ", e)
    return result


def own_rosnode_cleanup():
    pinged, unpinged = rosnode.rosnode_ping_all()
    if unpinged:
        master = rosgraph.Master(rosnode.ID)
        # noinspection PyTypeChecker
        rosnode.cleanup_master_blacklist(master, unpinged)


class ROSServer:
    def initROS(self):
        # ROS connection
        self.namespace_ros = NS
        self.topicsDict = {}
        self.servicesDict = {}
        self.actionsDict = {}
        rospy.init_node("rosopcua")

    def initOPCUA(self):
        # OPC-UA server
        self.server = opcua.Server()
        endpoint = "opc.tcp://{}:{}/".format(HOST,PORT)
        self.server.set_endpoint(endpoint)
        self.server.set_server_name("ROS ua Server")
        self.server.start()
        print('OPC-UA server listening on {}'.format(endpoint))
        
        # setup our own namespaces, this is expected
        self.uri_topics = "http://ros.org/topics"
        # two different namespaces to make getting the correct node easier for get_node (otherwise had object for service and topic with same name
        self.uri_services = "http://ros.org/services"
        self.uri_actions = "http://ros.org/actions"

        self.idx_topics = self.server.register_namespace(self.uri_topics)
        self.idx_services = self.server.register_namespace(self.uri_services)
        self.idx_actions = self.server.register_namespace(self.uri_actions)
        # get Objects node, this is where we should put our custom stuff
        objectsNode = self.server.get_objects_node()
        # one object per type we are watching
        self.topics_object = objectsNode.add_object(self.idx_topics, "ROS-Topics")
        self.services_object = objectsNode.add_object(self.idx_services, "ROS-Services")
        self.actions_object = objectsNode.add_object(self.idx_actions, "ROS-Actions")

    # Main loop
    def loop(self):
        try:
            while not rospy.is_shutdown():
                # TODO use topic and service listeners
                # ros_topics starts a lot of publisher/subscribers, might slow everything down quite a bit.
                rospy.logdebug('Checking for new services, topics and actions')
                ros_services.refresh_services(self.namespace_ros, self, self.servicesDict, self.idx_services, self.services_object)
                ros_topics.refresh_topics_and_actions(self.namespace_ros, self, self.topicsDict, self.actionsDict, self.idx_topics, self.idx_actions, self.topics_object, self.actions_object)
                # Don't clog cpu
                time.sleep(2)
        finally:
            # Always attempt to clean up
            self.cleanup()

    def cleanup(self, reason="Clean exit"):
        self.server.stop()
        rospy.signal_shutdown(reason)

    def find_service_node_with_same_name(self, name, idx):
        rospy.logdebug("Reached ServiceCheck for name " + name)
        for service in self.servicesDict:
            rospy.logdebug("Found name: " + str(self.servicesDict[service].parent.nodeid.Identifier))
            if self.servicesDict[service].parent.nodeid.Identifier == name:
                rospy.logdebug("Found match for name: " + name)
                return self.servicesDict[service].parent
        return None

    def find_topics_node_with_same_name(self, name, idx):
        rospy.logdebug("Reached TopicCheck for name " + name)
        for topic in self.topicsDict:
            rospy.logdebug("Found name: " + str(self.topicsDict[topic].parent.nodeid.Identifier))
            if self.topicsDict[topic].parent.nodeid.Identifier == name:
                rospy.logdebug("Found match for name: " + name)
                return self.topicsDict[topic].parent
        return None

    def find_action_node_with_same_name(self, name, idx):
        rospy.logdebug("Reached ActionCheck for name " + name)
        for topic in self.actionsDict:
            rospy.logdebug("Found name: " + str(self.actionsDict[topic].parent.nodeid.Identifier))
            if self.actionsDict[topic].parent.nodeid.Identifier == name:
                rospy.logdebug("Found match for name: " + name)
                return self.actionsDict[topic].parent
        return None


def main(args):
    server = ROSServer()
    server.initROS()
    server.initOPCUA()
    server.loop()


if __name__ == "__main__":
    main(sys.argv)
