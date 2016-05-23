from __future__ import with_statement 
import rospy
from std_msgs.msg import String
from roslaunch_axserver.msg import launchAction, launchGoal
from copy import copy, deepcopy
import actionlib

class LaunchWrapper(object):
    """docstring for LaunchWrapper"""
    def __init__(self, goal):
        super(LaunchWrapper, self).__init__()
        self.launch_goal = goal
        self.launch_client = actionlib.SimpleActionClient('launchServer', launchAction)
        if not self.launch_client.wait_for_server(timeout = rospy.Duration(60)):
            rospy.logfatal('Unable to connect to launch server')
            self.launch_client = None
            return 


        self.launch_client.send_goal(self.launch_goal, feedback_cb = self._feedback)

    def _feedback(self, feedback):
        self.ready = feedback.ready


    def tear_down(self):
        if self.launch_client is not None:
            self.launch_client.cancel_all_goals()            

class TearDownAtNodes(object):

    """ 

    """
    def __init__(self, tear_down_nodes, launch_files):
        super(TearDownAtNodes, self).__init__()
        
        self.tear_down_nodes = set(tear_down_nodes)        
        self.launch_goals = []
        self.launch_wrappers = []
        self.active_launch_files = []

        for lf in launch_files:
            goal = launchGoal()
            for k,v in lf.iteritems():
                setattr(goal, k, v)      

            for i in range(len(goal.values)):
                goal.values[i] = str(goal.values[i])
            self.launch_goals.append(goal)

        rospy.Subscriber('current_node', String, self._update_topological_location, queue_size = 1)


    def _launch_files_up(self):
        return len(self.launch_goals) == 0

    def _bring_up_launch_files(self):
        while len(self.launch_goals) > 0:
            self.launch_wrappers.append(LaunchWrapper(self.launch_goals.pop()))

    def _tear_down_launch_files(self):
        while len(self.launch_wrappers) > 0:
            wrapper = self.launch_wrappers.pop()
            wrapper.tear_down()
            self.launch_goals.append(wrapper.launch_goal)


    def _update_topological_location(self, node):        

        if node.data in self.tear_down_nodes:
            if self._launch_files_up():
                rospy.loginfo('Tearing down launch files at %s' % node.data)
                self._tear_down_launch_files()

        elif not self._launch_files_up():
            rospy.loginfo('Bringing launch files up at %s' % node.data)
            self._bring_up_launch_files()


