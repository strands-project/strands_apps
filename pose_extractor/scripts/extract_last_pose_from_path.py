#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

class PoseRepublisher():
    "A class to republish pose information"

    def __init__(self):
        rospy.init_node('pose_extractor_path')
        self.pub_topic = rospy.get_param("~pose", '/pose_extractor/pose')
        self.pub = rospy.Publisher(self.pub_topic, PoseStamped)
        self.sub_topic = rospy.get_param("~path", "/move_base/DWAPlannerROS/local_plan")
        rospy.Subscriber(self.sub_topic, Path, self.callback)
        rospy.logdebug(rospy.get_name() + " setting up")

    def callback(self,path):
        rospy.logdebug(rospy.get_name() + ": I heard %s" % path)
        self.pub.publish(path.poses[-1])

if __name__ == '__main__':
    republisher = PoseRepublisher()
    rospy.spin()
