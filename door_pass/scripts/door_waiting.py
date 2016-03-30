#! /usr/bin/env python

import rospy
import actionlib
from door_pass.msg import DoorWaitAction, DoorWaitResult
from door_pass.door_utils import DoorUtils
from mary_tts.msg import maryttsAction, maryttsGoal
   
class DoorWait(object):
    def __init__(self):
        max_trans_vel=rospy.get_param("~/max_trans_vel", 0.15)
        max_rot_vel=rospy.get_param("~/max_rot_vel", 0.4)
        vel_scale_factor=rospy.get_param("~/vel_scale_factor", 2)
        base_radius=rospy.get_param("~/base_radius", 0.31)
        getting_further_counter_threshold=rospy.get_param("~/getting_further_counter_threshold", 5)
        distance_to_success=rospy.get_param("~/distance_to_success", 0.2)
        
        self.door_utils=DoorUtils(max_trans_vel=max_trans_vel,
                                  max_rot_vel=max_rot_vel,
                                  vel_scale_factor=vel_scale_factor,
                                  base_radius=base_radius,
                                  getting_further_counter_threshold=getting_further_counter_threshold,
                                  distance_to_success=distance_to_success)
        
        
        self.door_as=actionlib.SimpleActionServer('door_wait', DoorWaitAction, execute_cb = self.execute_cb, auto_start=False) 
        self.door_as.start()
        
        self.waiting=False
        self.speaker = actionlib.SimpleActionClient('/speak', maryttsAction)
        

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
                                  distance_to_success=distance_to_success)
        
        target_pose=goal.target_pose.pose
        wait_timeout=goal.wait_timeout
        rospy.loginfo("Door wait and pass action server calling rotate towards pose")
        self.door_utils.rotate_towards_pose(target_pose)
        if self.door_as.is_preempt_requested():
            self.door_utils.deactivate()
            self.door_as.set_preempted()
            return
        self.waiting=True
        open_count=0
        wait_timer=rospy.Timer(rospy.Duration(wait_timeout), self.timer_cb, oneshot=True)
        while self.waiting and open_count<4:
            rospy.loginfo("Door wait and pass action server calling check door")
            door_open=self.door_utils.check_door(target_pose, 40)
            if door_open:
                open_count+=1
            else:
                open_count=0
            if self.door_as.is_preempt_requested():
                self.door_utils.deactivate()
                self.door_as.set_preempted()
                return
            rospy.sleep(rospy.Duration(0.5))
        wait_timer.shutdown()
      
        if open_count==4:
            #self.speaker.send_goal(maryttsGoal(text="I'm going to pass the door. Please hold it for me."))
            rospy.loginfo("The door is open.")
        else:
            rospy.loginfo("Timeout waiting for door to open.")
           
        self.door_utils.deactivate()        
        self.door_as.set_succeeded(result=DoorWaitResult(open=(open_count==4)))
    

    def timer_cb(self, event):
        self.waiting=False

    def main(self):
        rospy.spin()
    
if __name__ == '__main__':
    rospy.init_node("door_wait_node")
    waiter=DoorWait()
    waiter.main()
    
        
    
    
    
    

