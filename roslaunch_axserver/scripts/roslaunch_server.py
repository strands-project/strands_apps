#! /usr/bin/env python

import roslib
import rospy
import subprocess
import sys
import threading
import actionlib
import roslaunch_axserver.msg

roslib.load_manifest('roslaunch_axserver')


class RoslaunchServer(object):
    """
      create messages that are used to publish feedback/result
    """

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.ActionServer(
            self._action_name, roslaunch_axserver.msg.launchAction,
            self.execute_cb, self.cancel_cb, auto_start=False)
        self._as.start()
        self.p = {}
        # a lock to mutex access to the master object
        self.master_lock = threading.Lock()
        rospy.loginfo('Server is up')

    def check_topics(self, topics):
        rospy.logdebug('check topics')
        check = True
        # get_published_topics() appear to be not thread-safe, so
        # we mutex it here
        # see https://github.com/strands-project/strands_apps/issues/45
        with self.master_lock:
            existing_topics = [
                topic for [topic, type] in rospy.get_published_topics()]
        for t in topics:
            rospy.logdebug('check topic ' + t)
            check = check and (t in existing_topics)
        return check

    def monitor_thread_entry(self, gh, dummy):
        rospy.loginfo('monitor thread started for goal ' + gh.get_goal_id().id)
        _feedback = roslaunch_axserver.msg.launchFeedback()

        # accept the goal
        gh.set_accepted()
        rate = rospy.Rate(1)
        # check the relevant topics as long as this task is active
        while (gh.get_goal_status().status == actionlib.GoalStatus.ACTIVE and
               not rospy.is_shutdown()):
            rospy.logdebug('monitor thread working')

            # if process is still running
            if self.p[gh.get_goal_id()].poll() is None:
                try:
                    # if we have topics to be monitored:
                    if gh.get_goal().monitored_topics:
                        _feedback.ready = self.check_topics(
                            gh.get_goal().monitored_topics)
                    else:
                        _feedback.ready = True  # no monitors, so True!
                    gh.publish_feedback(_feedback)
                    rospy.logdebug('published feedback ' +
                                   str(_feedback.ready))
                except Exception, e:
                    rospy.logerr('Exception when checking monitored '
                                 'topics: %s. No feedback!' % e)
            else:
                if (self.p[gh.get_goal_id()].poll() == 0):
                    rospy.loginfo('launch process finished with success')
                    gh.set_succeeded()
                else:
                    rospy.loginfo('launch process finished with failure')
                    gh.set_aborted()
                return
            rate.sleep()
        rospy.loginfo('leave monitor thread for goal ' + gh.get_goal_id().id)

    def cancel_cb(self, gh):
        rospy.loginfo('cancel roslaunch goal ' + gh.get_goal_id().id)
        self.p[gh.get_goal_id()].terminate()
        gh.set_canceled()

    def execute_cb(self, gh):
        monitor_thread = threading.Thread(
            target=self.monitor_thread_entry, args=(gh, 1))
        rospy.loginfo('trigger roslaunch goal ' + gh.get_goal_id().id)
        goal = gh.get_goal()
        command = "exec roslaunch " + goal.pkg + " " + goal.launch_file
        command += "".join([" " + x + ":=" + y for (x, y) in zip(
          map(str.strip, goal.parameters), map(str.strip, goal.values))])
        try:
            self.p[gh.get_goal_id()] = subprocess.Popen(
                command, stdin=subprocess.PIPE, shell=True)
            rospy.loginfo(self.p[gh.get_goal_id()].pid)
            monitor_thread.start()

        except:
            print "Unexpected error:", sys.exc_info()[0]
            gh.set_rejected()


if __name__ == '__main__':
    rospy.init_node('launchServer')
    RoslaunchServer(rospy.get_name())
    rospy.spin()
