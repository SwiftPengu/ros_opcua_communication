import rospy

def refresh_dict(namespace_ros, actionsdict, topicsdict, server, idx_actions):
    topics = rospy.get_published_topics(namespace_ros)
    tobedeleted = []
    for actionNameOPC in actionsdict:
        found = False
        for topicROS, topic_type in topics:
            if actionNameOPC in topicROS:
                found = True
        if not found:
            actionsdict[actionNameOPC].recursive_delete_items(actionsdict[actionNameOPC].parent)
            tobedeleted.append(actionNameOPC)
            rospy.logdebug("deleting action: " + actionNameOPC)
    for name in tobedeleted:
        del actionsdict[name]