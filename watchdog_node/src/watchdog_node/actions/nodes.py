from . import ActionType
import rospy
import rosnode

class KillNode(ActionType):
    name = "KillNode"
    description = "Sends a specified node a kill signal."
    config_keys = [("node_name", "The rosgraph name of the node to kill.")]
    
    def execute(self):
        rospy.logwarn("Watchdog killing node {}".format(self.node_name))
        killed = rosnode.kill_nodes([self.node_name])
        