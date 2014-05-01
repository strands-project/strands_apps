#! /usr/bin/env python

import roslib
roslib.load_manifest('roslaunch_axserver')
import rospy
import subprocess
import sys

import actionlib

import roslaunch_axserver.msg


class RoslaunchServer(object):
# create messages that are used to publish feedback/result
    _feedback = roslaunch_axserver.msg.launchFeedback()
    _result = roslaunch_axserver.msg.launchResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.ActionServer(
            self._action_name, roslaunch_axserver.msg.launchAction,
            self.execute_cb, self.cancel_cb)
        self._as.start()
        self.p = {}
        rospy.loginfo('Server is up')

    def cancel_cb(self, gh):
        rospy.loginfo('cancel roslaunch')
        self.p[gh.get_goal_id()].terminate()
        gh.set_canceled()

    def execute_cb(self, gh):
        rospy.loginfo('call roslaunch')
        goal = gh.get_goal()
        command = "exec roslaunch " + goal.pkg + " " + goal.launch_file
        try:
            self.p[gh.get_goal_id()] = subprocess.Popen(
                command, stdin=subprocess.PIPE, shell=True)
            rospy.loginfo(self.p[gh.get_goal_id()].pid)
            gh.set_accepted()
        except:
            print "Unexpected error:", sys.exc_info()[0]
            gh.set_rejected()


if __name__ == '__main__':
    rospy.init_node('launchServer')
    RoslaunchServer(rospy.get_name())
    rospy.spin()
