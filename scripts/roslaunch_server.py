#! /usr/bin/env python

import roslib
roslib.load_manifest('roslaunch_axserver')
import rospy
import subprocess

import actionlib

import roslaunch_axserver.msg


class RoslaunchServer(object):
# create messages that are used to publish feedback/result
    _feedback = roslaunch_axserver.msg.launchFeedback()
    _result = roslaunch_axserver.msg.launchResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(
            self._action_name, roslaunch_axserver.msg.launchAction,
            execute_cb=self.execute_cb)
        self._as.start()
        rospy.loginfo('Server is up')

    def execute_cb(self, goal):
        rospy.loginfo('call roslaunch')
        command = "exec roslaunch strands_morse uol_mht_nav2d.launch"
        command = "exec xev"
        self.p = subprocess.Popen(
            command, stdin=subprocess.PIPE, shell=True)
        rospy.loginfo(self.p.pid)
        # process = p
        # print p.stdout.read()
        # check if the goal is preempted
        while 1:
            if self._as.is_preempt_requested():
                rospy.loginfo('Logging is preempted')
                self.p.terminate()
                #self.p.send_signal(3)
                self._as.set_preempted()
                break


if __name__ == '__main__':
    rospy.init_node('launchServer')
    RoslaunchServer(rospy.get_name())
    rospy.spin()
