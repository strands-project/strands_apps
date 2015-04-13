#! /usr/bin/env python

import rospy
import actionlib
from actionlib_msgs.msg import GoalStatusArray, GoalStatus
from move_base_msgs.msg import MoveBaseAction
from door_pass.door_utils import DoorUtils
import dynamic_reconfigure.client


class DoorPass(object):

    def __init__(self):
        self.door_as=actionlib.SimpleActionServer('doorPassing', MoveBaseAction, execute_cb = self.execute_cb, auto_start=False) 
        self.door_as.start()
        
        self.mon_nav_status_sub=rospy.Subscriber("/monitored_navigation/status", GoalStatusArray, self.mon_nav_status_cb)
        self.mon_nav_executing=False

    def mon_nav_status_cb(self, data):
        self.mon_nav_executing=data.status_list!=[] and data.status_list[0].status==GoalStatus.ACTIVE

    def execute_cb(self, goal):

        default_speed=rospy.get_param("~/default_speed", 0.15)
        base_radius=rospy.get_param("~/base_radius", 0.31)
        getting_further_counter_threshold=rospy.get_param("~/getting_further_counter_threshold", 5)
        distance_to_success=rospy.get_param("~/distance_to_success", 0.2)
        
        # read speed limits frmo move base
        client = dynamic_reconfigure.client.Client('/move_base/DWAPlannerROS', timeout=5)
        config = client.get_configuration(timeout=5)

        # these are taken from the current strands movebase config
        if config is None:
            config = {'min_vel_x': 0.0, 'min_vel_y': 0.0,  'min_rot_vel': 0.4, 'acc_lim_x': 1.0, 'acc_lim_y': 0.0, 'acc_lim_theta': 3.2, 'max_rot_vel': 1.0, 'max_vel_x': 0.55, 'max_vel_y': 0.0}

        self.door_utils=DoorUtils(default_speed=default_speed, base_radius=base_radius, getting_further_counter_threshold=getting_further_counter_threshold, distance_to_success=distance_to_success, move_base_cfg=config)
        

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
        rospy.loginfo("Door pass action server calling check door")
        door_open=self.door_utils.check_door(target_pose)
        if door_open:
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
            rospy.loginfo("Door is closed. Disabling monitored navigation recoveries.")
            current_mon_nav_recover_states=rospy.get_param("/monitored_navigation/recover_states/", {})
            for mon_nav_recover_state, value in current_mon_nav_recover_states.iteritems():
                rospy.set_param("/monitored_navigation/recover_states/" + mon_nav_recover_state, [False, value[1]])
            self.door_as.set_aborted()
            #wait for mon nav to output failure and get recover states back on
            while self.mon_nav_executing:
                rospy.loginfo("Waiting for monitored navigation to stop executing")
                rospy.sleep(0.1)
            rospy.loginfo("Monitored navigation stopped executing. Resetting monitored navigation recoveries.")
            rospy.set_param("/monitored_navigation/recover_states/", current_mon_nav_recover_states)
            return



    def main(self):
        rospy.spin()

    
if __name__ == '__main__':
    rospy.init_node("door_pass_node")
    passer=DoorPass()
    passer.main()
    
        
    
    
    
    

