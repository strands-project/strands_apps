#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry

class OdometryRepublisher():
	"A class to republish joint state information"

	def __init__(self):
		rospy.init_node('odometry_republisher')
		self.pub = rospy.Publisher('odom', Odometry)
                rospy.Subscriber("odom_morse", Odometry, self.callback)
		rospy.loginfo(rospy.get_name() + " setting up")

	def callback(self,data):
                rospy.logdebug(rospy.get_name() + ": I heard %s %s, %s",  data.twist.twist.linear.x, data.twist.twist.linear.y,  data.twist.twist.linear.z)
                
                odom = data
                odom.pose.pose.position.z = 0.0
                odom.twist.twist.linear.y = 0.0
                odom.twist.twist.linear.z = 0.0
                odom.twist.twist.angular.x = 0.0
                odom.twist.twist.angular.y = 0.0
                self.pub.publish(odom)

if __name__ == '__main__':
    republisher = OdometryRepublisher()
    rospy.spin()
