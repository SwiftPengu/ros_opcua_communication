# !/usr/bin/python
# thanks to https://github.com/ros-visualization/rqt_common_plugins/blob/groovy-devel/rqt_action/src/rqt_action/action_plugin.py
import rospy
import os.path as path

# Get topic name from full action_name
def get_correct_name(topic_name):
    rospy.logdebug("getting correct name for: " + str(topic_name))
    return path.dirname(topic_name)