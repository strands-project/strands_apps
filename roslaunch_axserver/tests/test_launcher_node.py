#!/usr/bin/env python
from __future__ import division

PKG = 'roslaunch_axserver'
NAME = 'roslaunch_axserver_tester'

import rospy
import unittest
import rostest
import sys
from math import ceil

from random import random


class TestEntry(unittest.TestCase):
# class TestEntry():

    def test_launch(self):
        import roslaunch_axserver.msg
        import actionlib
        client = actionlib.SimpleActionClient(
            'roslaunch_axserver', roslaunch_axserver.msg.launchAction)

        # Waits until the action server has started up and started
        # listening for goals.
        client.wait_for_server()

        # Creates a goal to send to the action server.
        goal = roslaunch_axserver.msg.launchGoal(
            pkg='roslaunch_axserver', launch_file='test-launch.xml', monitored_topics=[])

        # Sends the goal to the action server.
        client.send_goal(goal)
        # Waits for the server to finish performing the action.
        client.wait_for_result()
  
        # Prints out the result of executing the action
        res = client.get_state()  # 
        self.assertEqual(res, 3)

if __name__ == '__main__':
    rospy.init_node(NAME)
    rostest.rosrun(PKG, NAME, TestEntry, sys.argv)
    # test = TestEntry()
    # test.test_start_at_now()

    # rospy.spin()

