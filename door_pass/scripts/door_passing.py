#! /usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction
from door_pass.door_utils import DoorUtils

class DoorPass(object):

    def __init__(self):
        default_speed=rospy.get_param("~/default_speed", 0.15)
        base_radius=rospy.get_param("~/base_radius", 0.31)
        getting_further_counter_threshold=rospy.get_param("~/getting_further_counter_threshold", 5)
        distance_to_success=rospy.get_param("~/distance_to_success", 0.2)
        self.door_utils=DoorUtils(default_speed=default_speed, base_radius=base_radius, getting_further_counter_threshold=getting_further_counter_threshold, distance_to_success=distance_to_success)
        self.door_as=actionlib.SimpleActionServer('doorPassing', MoveBaseAction, execute_cb = self.execute_cb, auto_start=False) 
        self.door_as.start()

    def execute_cb(self, goal):
        if self.door_as.is_preempt_requested():
            self.door_as.set_preempted()
            return
        target_pose=goal.target_pose.pose
        rospy.loginfo("Door pass action server calling rotate towards pose")
        self.door_utils.rotate_towards_pose(target_pose)
        if self.door_as.is_preempt_requested():
            self.door_as.set_preempted()
            rospy.loginfo("Door pass action preempted")
            return
        #put recover states off
        rospy.loginfo("Door pass action server calling check door")
        door_open=self.door_utils.check_door(target_pose)
        if door_open:
            #put recover states on
            if self.door_as.is_preempt_requested():
                self.door_as.set_preempted()
                rospy.loginfo("Door pass action preempted")
                return
            rospy.loginfo("The door is open. Door pass action server is calling pass door")
            success=self.door_utils.pass_door(target_pose)
            if success:
                self.door_as.set_succeeded()
                return
            else:
                self.door_as.set_aborted()
                return
        else:
            self.door_as.set_aborted()
            #wait for mon nav to output failure and get recover states back on
            return



    def main(self):
        rospy.spin()

    
if __name__ == '__main__':
    rospy.init_node("door_pass_node")
    passer=DoorPass()
    passer.main()
    
        
    
    
    
    

