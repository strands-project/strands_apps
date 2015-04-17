#! /usr/bin/env python

import rospy
import actionlib
from door_pass.msg import DoorCheckAction, DoorCheckResult
from door_pass.door_utils import DoorUtils
   
class DoorCheck(object):
    def __init__(self):    
        self.door_as=actionlib.SimpleActionServer('door_check', DoorCheckAction, execute_cb = self.execute_cb, auto_start=False) 
        self.door_as.start()
        rospy.loginfo("Door check action server initialised")
        
    def execute_cb(self, goal):
        door_utils=DoorUtils(max_trans_vel=0,
                                  max_rot_vel=0,
                                  vel_scale_factor=0,
                                  base_radius=0,
                                  getting_further_counter_threshold=0,
                                  distance_to_success=0)
        door_utils.activate()
        door_open=door_utils.check_door()
        if door_open:
            rospy.loginfo("The door is open")
        else:
            rospy.loginfo("The door is closed")
        door_utils.deactivate()        
        self.door_as.set_succeeded(result=DoorCheckResult(open=door_open))
          
    def main(self):
        rospy.spin()
    
if __name__ == '__main__':
    rospy.init_node("door_pass_node")
    checker=DoorCheck()
    checker.main()
    