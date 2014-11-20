#!/usr/bin/env python

import rospy

import smtplib

from email.mime.multipart import MIMEMultipart
from email.mime.text import MIMEText

import actionlib
import strands_emails.msg

class emailServer(object):

    _feedback = strands_emails.msg.SendEmailFeedback()
    _result   = strands_emails.msg.SendEmailResult()

    def __init__(self, name):
        self.cancelled = False
        self._action_name = name

        self.from_add = rospy.get_param("~from_add",'robot@strands.eu')
        self.smtp_add = rospy.get_param("~smtp_add",'localhost:25')


        rospy.loginfo("Creating action server.")
        self._as = actionlib.SimpleActionServer(self._action_name, strands_emails.msg.SendEmailAction, execute_cb = self.executeCallback, auto_start = False)
        self._as.register_preempt_callback(self.preemptCallback)
        rospy.loginfo(" ...starting")
        self._as.start()
        rospy.loginfo(" ...done")

        rospy.loginfo("Ready to Tweet ...")
        rospy.spin()

    def _send_email(self, goal):
        self.cancelled = False
        me = self.from_add
        msg = MIMEMultipart('alternative')
        msg['Subject'] = goal.subject
        msg['From'] = me
        msg['To'] = goal.to_address
        texts = "%s"%goal.text
        part1 = MIMEText(texts, 'plain')
        msg.attach(part1)
        
        server = smtplib.SMTP(self.smtp_add)
        
        server.sendmail(me, goal.to_address, msg.as_string())
        server.quit()
        return True


    def executeCallback(self, goal):
        self._feedback.senttext = 'Sending...'
        self._as.publish_feedback(self._feedback)
        rospy.loginfo('%s: sending %s' % (self._action_name, goal.text))
        result=self._send_email(goal)
        self._result.success = result
        self._feedback.senttext = goal.text
        self._as.publish_feedback(self._feedback)
        self._as.set_succeeded(self._result)


    def preemptCallback(self):
        self.cancelled = True
        self._result.success = False
        self._as.set_preempted(self._result)
        

if __name__ == '__main__':
    rospy.init_node('strands_emails')
    server = emailServer(rospy.get_name())