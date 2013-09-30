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
		self.emailSend = False
	
    		rospy.init_node('state_checker', anonymous=True)
		rospy.Subscriber("odom", Odometry, self.callback)

		self.sender = rospy.get_param('~sender')
		self.sender_password = rospy.get_param('~password')
		self.receiver = rospy.get_param('~receiver')
		self.email_title = rospy.get_param('~email_title')
		self.email_body = rospy.get_param('~email_body')
		self.email_attachment= rospy.get_param('~email_attachment','')
		self.odometry_timeout = float(rospy.get_param('~timeout',90.0)) # seconds

		print self.sender, self.sender_password, self.receiver, self.email_title, self.email_body


	def callback(self,data):
		if (data.pose.pose.position.x != self.prevPose.position.x):
			self.prevPose = data.pose.pose
        		rospy.loginfo(rospy.get_name() + ": I heard %s" % data.pose.pose)
			self.emailSent = False
			self.start_time = time.time()
		else :
			elapsedTime = time.time() - self.start_time
			if (elapsedTime > self.odometry_timeout):
				if (not self.emailSent):
					print "I haven't moved in ",elapsedTime," seconds. HELP!!!!"
					sendmail(self.sender, self.sender_password,self.receiver, self.email_title,self.email_body,self.email_attachment)
					self.emailSent = True
				

if __name__ == '__main__':
    state_checker = stateChecker()
    rospy.spin()
