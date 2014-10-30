#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Vector3 
from sensor_msgs.msg import JointState

class JointStateRepublisher():
	"A class to republish joint state information"

	def __init__(self):
		rospy.init_node('jointstate_republisher')
		self.pub = rospy.Publisher('/ptu', Vector3)
                rospy.Subscriber("/ptu/cmd", JointState, self.callback)
		rospy.loginfo(rospy.get_name() + " setting up")

	def callback(self,data):
                rospy.loginfo(rospy.get_name() + ": I heard %s, %s", data.name, data.position)
                
                pan_idx = data.name.index('pan')
                tilt_idx = data.name.index('tilt')
                
                vec3 = Vector3()
    
                vec3.x = 0.0
                vec3.y = data.position[pan_idx]
                vec3.z = data.position[tilt_idx]

                self.pub.publish(vec3)

if __name__ == '__main__':
    republisher = JointStateRepublisher()
    rospy.spin()
