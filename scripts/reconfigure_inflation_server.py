#! /usr/bin/env python

import rospy
from move_base_msgs.msg import *
import dynamic_reconfigure.client
from actionlib_msgs.msg import *
import actionlib

class ReconfigureInflationServer(object):
    def __init__(self):
        rospy.init_node('reconfigure_inflation_server')
        
        self.move_base_action_client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        self.local_reconfig_client = dynamic_reconfigure.client.Client('/move_base/local_costmap/inflation_layer')
        self.global_reconfig_client = dynamic_reconfigure.client.Client('/move_base/global_costmap/inflation_layer')
        
        #self.feedback = BacktrackFeedback()
        #self.result = BacktrackResult()
        
        self.server = actionlib.SimpleActionServer('move_base_inflated', MoveBaseAction, self.execute, False)
        self.server.start()
        
    def reset_move_base_pars(self):
        local_params = { 'inflation_radius' : self.prev_local_inflation }
        global_params = { 'inflation_radius' : self.prev_global_inflation }
        local_config = self.local_reconfig_client.update_configuration(local_params)
        global_config = self.local_reconfig_client.update_configuration(global_params)
        
    def execute(self, goal):    
        self.prev_local_inflation = rospy.get_param("/move_base/local_costmap/inflation_layer/inflation_radius")
        self.prev_global_inflation = rospy.get_param("/move_base/global_costmap/inflation_layer/inflation_radius")
        local_params = { 'inflation_radius' : 0.8 }
        global_params = { 'inflation_radius' : 0.8 }
        local_config = self.local_reconfig_client.update_configuration(local_params)
        global_config = self.local_reconfig_client.update_configuration(global_params)
        
        print "Set the inflation radius to 0.8"
        
        self.move_base_action_client.send_goal(goal)
        status = self.move_base_action_client.get_state()
        while status == GoalStatus.PENDING or status == GoalStatus.ACTIVE:
            status = self.move_base_action_client.get_state()
            if self.server.is_preempt_requested():
                self.reset_move_base_pars()
                self.move_base_action_client.cancel_goal()
                self.server.set_preempted(self.move_base_action_client.get_result())
                return
            self.move_base_action_client.wait_for_result(rospy.Duration(0.2))
        
        print "Reset the inflation radius to ", self.prev_local_inflation
        
        self.reset_move_base_pars()
        if status == GoalStatus.PREEMPTED:
            self.server.set_preempted(self.move_base_action_client.get_result())
        elif status == GoalStatus.SUCCEEDED:
            self.server.set_succeeded(self.move_base_action_client.get_result())
        else
            self.server.set_aborted(self.move_base_action_client.get_result())

if __name__ == '__main__':
    server = ReconfigureInflationServer()
    rospy.spin()
