#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from sendemailscript import sendmail 
import time


class stateChecker:
	def __init__(self):
		self.start_time = time.time()
		self.prevPose = Pose()
		self.timeout_time = 90 # seconds
		self.emailSend = False
	
    		rospy.init_node('state_checker', anonymous=True)
		rospy.Subscriber("odom", Odometry, self.callback)

	def callback(self,data):
		if (data.pose.pose.position.x != self.prevPose.position.x):
			self.prevPose = data.pose.pose
        		rospy.loginfo(rospy.get_name() + ": I heard %s" % data.pose.pose)
			self.emailSent = False
			self.start_time = time.time()
		else :
			elapsedTime = time.time() - self.start_time
			if (elapsedTime > self.timeout_time):
				if (not self.emailSent):
					print "I haven't moved in ",elapsedTime," seconds. HELP!!!!"
					sendmail("rosiethesmartrobot@gmail.com", "rosie@123",["nbore@kth.se", "raambrus@kth.se"], "I think I'm stuck", "HELP!!!\n\nI haven't moved in very long, please come take a look.","")
					self.emailSent = True
				

if __name__ == '__main__':
    state_checker = stateChecker()
    rospy.spin()
