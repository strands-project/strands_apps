#! /usr/bin/env python

import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
from door_pass.msg import DoorWaitAction, DoorWaitResult
from door_pass.door_utils import DoorUtils
   
class DoorWait(object):
    def __init__(self):
        max_trans_vel=rospy.get_param("~max_trans_vel", 0.15)
        max_rot_vel=rospy.get_param("~max_rot_vel", 0.4)
        vel_scale_factor=rospy.get_param("~vel_scale_factor", 2)
        base_radius=rospy.get_param("~base_radius", 0.31)
        getting_further_counter_threshold=rospy.get_param("~getting_further_counter_threshold", 5)
        distance_to_success=rospy.get_param("~distance_to_success", 0.2)
        n_closed_door=rospy.get_param("~n_closed_door", 40)  
        
        
        self.door_utils=DoorUtils(max_trans_vel=max_trans_vel,
                                  max_rot_vel=max_rot_vel,
                                  vel_scale_factor=vel_scale_factor,
                                  base_radius=base_radius,
                                  getting_further_counter_threshold=getting_further_counter_threshold,
                                  distance_to_success=distance_to_success,
                                  n_closed_door = n_closed_door)
        
        
        self.door_as=actionlib.SimpleActionServer('door_wait', DoorWaitAction, execute_cb = self.execute_cb, auto_start=False) 
        self.door_as.start()
        self.door_as.register_preempt_callback(self.door_as_preempt_cb)  
        
        self.waiting=False
        

    def execute_cb(self, goal):
        self.door_utils.activate()
        max_trans_vel=rospy.get_param("~/max_trans_vel", 0.15)
        max_rot_vel=rospy.get_param("~/max_rot_vel", 0.4)
        vel_scale_factor=rospy.get_param("~/vel_scale_factor", 2)
        base_radius=rospy.get_param("~/base_radius", 0.31)
        getting_further_counter_threshold=rospy.get_param("~/getting_further_counter_threshold", 5)
        distance_to_success=rospy.get_param("~/distance_to_success", 0.2)        
        self.door_utils.set_params(max_trans_vel=max_trans_vel,
                                  max_rot_vel=max_rot_vel,
                                  vel_scale_factor=vel_scale_factor,
                                  base_radius=base_radius,
                                  getting_further_counter_threshold=getting_further_counter_threshold,
                                  distance_to_success=distance_to_success,
                                  n_closed_door = n_closed_door)
        
        target_pose=goal.target_pose.pose
        wait_timeout=goal.wait_timeout
        rospy.loginfo("Door wait and pass action server calling rotate towards pose")
        self.door_utils.rotate_towards_pose(target_pose)
        
        if self.door_as.is_preempt_requested():
            self.finish_execution(GoalStatus.PREEMPTED)
            return
        
        opened=self.door_utils.wait_door(wait_timeout, target_pose, 40)
        
        if self.door_as.is_preempt_requested():
            self.finish_execution(GoalStatus.PREEMPTED)
        else:  
            self.finish_execution(GoalStatus.SUCCEEDED, opened)
    
    def door_as_preempt_cb(self):
        self.door_utils.deactivate()
        
    def finish_execution(self, status, result=None):
        rospy.loginfo("Door waiting finished with outcome " + GoalStatus.to_string(status))
        self.door_utils.deactivate()
        if status==GoalStatus.SUCCEEDED:
            self.door_as.set_succeeded(result=DoorWaitResult(open=result))
        if status==GoalStatus.PREEMPTED:
            self.door_as.set_preempted()


    def timer_cb(self, event):
        self.waiting=False

    def main(self):
        rospy.spin()
    
if __name__ == '__main__':
    rospy.init_node("door_wait_node")
    waiter=DoorWait()
    waiter.main()
    
        
    
    
    
    

