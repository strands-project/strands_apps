#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState

class JointStateRepublisher():
	"A class to republish joint state information"

	def __init__(self):
		rospy.init_node('ptu_state_republisher')
		self.pub = rospy.Publisher('/ptu/state', JointState)
                rospy.Subscriber("/ptu_state", JointState, self.callback)
		rospy.loginfo(rospy.get_name() + " setting up")

	def callback(self,data):
                rospy.logdebug(rospy.get_name() + ": I heard %s, %s", data.name, data.position)
                
                pan_idx = data.name.index('pan')
                tilt_idx = data.name.index('tilt')
                
                js = JointState()

                js.header = data.header
                js.name.append(data.name[pan_idx])
                js.name.append(data.name[tilt_idx])
                js.position.append(data.position[pan_idx])
                js.position.append(data.position[tilt_idx])

                self.pub.publish(js)

if __name__ == '__main__':
    republisher = JointStateRepublisher()
    rospy.spin()
