#! /usr/bin/env python

import rospy
import actionlib
from actionlib_msgs.msg import GoalStatusArray, GoalStatus
from move_base_msgs.msg import MoveBaseAction
from door_pass.door_utils import DoorUtils

   
class DoorWaitAndPass(object):
    def __init__(self):
        max_trans_vel=rospy.get_param("~/max_trans_vel", 0.15)
        max_rot_vel=rospy.get_param("~/max_rot_vel", 0.4)
        vel_scale_factor=rospy.get_param("~/vel_scale_factor", 2)
        base_radius=rospy.get_param("~/base_radius", 0.31)
        getting_further_counter_threshold=rospy.get_param("~/getting_further_counter_threshold", 5)
        distance_to_success=rospy.get_param("~/distance_to_success", 0.2)
        self.wait_timeout=rospy.get_param("~/wait_timeout", 60)
        self.stand_alone=rospy.get_param("~/do_waiting", False)
        
        self.door_utils=DoorUtils(max_trans_vel=max_trans_vel,
                                  max_rot_vel=max_rot_vel,
                                  vel_scale_factor=vel_scale_factor,
                                  base_radius=base_radius,
                                  getting_further_counter_threshold=getting_further_counter_threshold,
                                  distance_to_success=distance_to_success)
        
        self.mon_nav_status_sub=rospy.Subscriber("/monitored_navigation/status", GoalStatusArray, self.mon_nav_status_cb)
        
        self.door_as=actionlib.SimpleActionServer('door_wait_and_pass', MoveBaseAction, execute_cb = self.execute_cb, auto_start=False) 
        self.door_as.start()
        self.door_as.register_preempt_callback(self.door_as_preempt_cb)        
        self.mon_nav_executing=False        

    def mon_nav_status_cb(self, data):
        result=False
        for goal in data.status_list:
            if goal.status==GoalStatus.ACTIVE:
                result=True
                break
        self.mon_nav_executing=result

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
        rospy.loginfo("Door wait and pass action server calling rotate towards pose")
        self.door_utils.rotate_towards_pose(target_pose)
        if self.door_as.is_preempt_requested():
            self.finish_execution(GoalStatus.PREEMPTED)
            return

        if self.stand_alone:
            consecutive_opens=5
        else:
            consecutive_opens=1
        opened=self.door_utils.wait_door(self.wait_timeout, target_pose, 40, False, self.stand_alone, consecutive_opens)
        
        if self.door_as.is_preempt_requested():
            self.finish_execution(GoalStatus.PREEMPTED)
            return

        if opened:
            rospy.loginfo("The door is open. Door wait and pass action server is calling pass door")
            success=self.door_utils.pass_door(target_pose, speech=True)
            if self.door_as.is_preempt_requested():
                self.finish_execution(GoalStatus.PREEMPTED)
                return
            if success:
                self.finish_execution(GoalStatus.SUCCEEDED)
                return
            else:
                self.finish_execution(GoalStatus.ABORTED)
                return
        else:
            rospy.loginfo("Timeout waiting for door to open. Disabling monitored navigation recoveries.")
            current_mon_nav_recover_states=rospy.get_param("/monitored_navigation/recover_states/", {})
            for mon_nav_recover_state in current_mon_nav_recover_states:
                rospy.set_param("/monitored_navigation/recover_states/" + mon_nav_recover_state, [False,0])
            self.finish_execution(GoalStatus.ABORTED)
            #wait for mon nav to output failure and get recover states back on
            timeout=0
            while self.mon_nav_executing and not self.door_as.is_preempt_requested() and timeout<30:
                rospy.loginfo("Waiting for monitored navigation to stop executing")
                rospy.sleep(0.1)
                timeout=timeout+1
            rospy.loginfo("Monitored navigation stopped executing. Resetting monitored navigation recoveries.")
            rospy.set_param("/monitored_navigation/recover_states/", current_mon_nav_recover_states)            
            return
    
    def finish_execution(self, status):
        rospy.loginfo("Door passing finished with outcome " + GoalStatus.to_string(status))
        self.door_utils.deactivate()
        if status==GoalStatus.SUCCEEDED:
            self.door_as.set_succeeded()
        if status==GoalStatus.ABORTED:
            self.door_as.set_aborted()
        if status==GoalStatus.PREEMPTED:
            self.door_as.set_preempted()
        

    def door_as_preempt_cb(self):
        self.door_utils.deactivate()


    def main(self):
        rospy.spin()
    
if __name__ == '__main__':
    rospy.init_node("door_wait_and_pass_node")
    passer=DoorWaitAndPass()
    passer.main()
    
        
    
    
    
    

