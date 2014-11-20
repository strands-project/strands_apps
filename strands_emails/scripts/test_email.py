#! /usr/bin/env python

import rospy
import sys
# Brings in the SimpleActionClient
import actionlib
import strands_emails.msg


def email_client(text):
    client = actionlib.SimpleActionClient('strands_emails', strands_emails.msg.SendEmailAction)
    client.wait_for_server()
    goal = strands_emails.msg.SendEmailGoal()


    rospy.loginfo(" ... Init done")
    goal.text = text    
    goal.to_address = sys.argv[1]
    goal.subject = sys.argv[2]

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()  # A FibonacciResult

if __name__ == '__main__':
    print 'Argument List:',str(sys.argv)
    text0 = " ".join(sys.argv[3:])
    rospy.init_node('email_test_py')
    print "Sending %s"%(text0)
    ps = email_client(text0)
    print ps