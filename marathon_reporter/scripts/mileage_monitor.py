#!/usr/bin/python
import rospy
from nav_msgs.msg import Odometry
from mongodb_store import message_store
from math  import sqrt
from marathon_reporter.msg import MarathonSession

import requests
REQUESTS_ECEPTIONS = tuple([requests.exceptions.__getattribute__(i) 
                     for i in dir(requests.exceptions) if not i.startswith("_")])
print REQUESTS_ECEPTIONS
import sys
import hashlib
import yaml
import os
import threading

BASE_URL = "http://www.cs.bham.ac.uk/~burbrcjc/marathon2014/"
REPORT_URL =  BASE_URL + "report.php"
TIME_INTERVAL = 5 * 60

class MarathonReporter(object):
    def __init__(self):
        rospy.init_node("strands_marathon_reporter", anonymous=False)
        self._load_user_details()
        self._team_hash =  hashlib.md5(self._team_name).hexdigest()
        self._msg_store =  message_store.MessageStoreProxy(collection="marathon2014")
        
        # Get the initial odometry pose
        rospy.loginfo("Waiting for robot odometry...")
        try:
            self._last_point = rospy.wait_for_message("/odom", Odometry, 10).pose.pose.position
        except:
            rospy.logerr("Can't get an odometry message. Please make sure your robot is "
                         "publishing standard nav_msgs/Odometry messages on the /odom topic." )
            self._die()

        # Initialise a new session
        rospy.loginfo("Starting a new marathon session.")
        try:
            self._initialise_session()
        except REQUESTS_EXCEPTIONS,  e:
            rospy.logerr("Connection error. Internet must be available at startup"
                         " even if it is only available intermitently during run."
                         " Terminating reporting, this run will not be counted!.")
            rospy.logerr("Python requests message: %s" % str(e))
            self._die()
        self._odom_sub =  rospy.Subscriber("/odom", Odometry,
                                           self._odometry_cb)
        
        rospy.on_shutdown(self._shutdown_cb)
        
        self._reporting = True
        self._timout =  threading.Timer(1, self.report_session)
        self._timout.start()
        
    def _load_user_details(self):
        try:
            with open(os.path.expanduser("~/.marathon_auth"), "r") as f:
                creds = yaml.load(f.read())
            print creds
            self._team_name = creds["team"]
            self._passkey = creds["password"]
        except:
            rospy.logerr("Error reading credentials file ~/.marathon_auth\n"
                         "Please check the file contains your user details "
                         "and is readable")
            self._die()
        
    def _web_get(self, session_id, distance=None, duration=0, end=0):
        hash_string = requests.get(BASE_URL+self._team_hash).text.strip()
        auth =  hashlib.md5(hash_string + self._passkey).hexdigest()
        params = {'id': self._team_name,
                  'auth': auth,
                  'runid': session_id,
                  'distance': distance,
                  'duration': duration, 
                  'end': end,}
        request = requests.get(REPORT_URL, params=params)
        rt = request.text.strip()
        if rt.startswith("err"):
            rospy.logerr("Web reporting error code %s"%rt[3])
            if rt[3] == "0":
                rospy.logerr("Username/password incorrect. Please verify your"
                             " setup. Your username and password should be "
                             "set in ~/.marathon_auth")
            self._die()
        else:
            return request.text.strip()
        
    def _initialise_session(self):
        # Locally end all historic sessions and ensure web knows about them
        try:
            sessions = self._msg_store.query(MarathonSession._type,
                                             message_query={'team': self._team_name,})
            for session, meta in sessions:
                rospy.loginfo("Found historic session %s"%session.name)
                if not session.ended:
                    rospy.loginfo("->session not ended,  marking end and submitting")
                    # The session is a dangler, end it and let the web know
                    end = self._web_get(session.name, session.distance,
                                  session.duration.to_sec(), end=1)
                    if end == "ok":
                        session.ended = True
                        self._msg_store.update(session,
                                               message_query={'name': session.name,})
                    else:
                        rospy.logwarn("Could not mark sesion ended on web, response=%s"%end)


            # Get a new session id from the web session logger
            self._current_session = MarathonSession()
            self._current_session.team =  self._team_name
            self._current_session.name = self._web_get(session_id="new")
            self._current_session.duration = rospy.Duration(0.0)
            self._current_session.distance = 0.0
            self._current_session.ended = False
            self._local_start_time =  rospy.Time.now()

            # Save it in the database
            self._msg_store.insert(self._current_session)

            rospy.loginfo("New marathon session started, id=%s." %
                          self._current_session.name)
        except rospy.ServiceException, e:
            rospy.logerror("Problem calling mongodb_store services. mongodb_store is required "
                           "by the marathon reporter.")
            self._die()

        
    def report_session(self):
        rospy.loginfo("Reporting session duration of %ds, distance of %dm" %
                      (self._current_session.duration.to_sec(),
                       self._current_session.distance))
        try:
            self._web_get(session_id=self._current_session.name,
                          distance=self._current_session.distance,
                          duration=self._current_session.duration.to_sec())
        except REQUESTS_EXCEPTIONS,  e:
            rospy.logwarn("Interim reporting skipped due to connection error.")
        
        self._timout =  threading.Timer(TIME_INTERVAL, self.report_session)
        if self._reporting:
            self._timout.start()
    
    def _odometry_cb(self, odom):
        distance =  sqrt((odom.pose.pose.position.x - self._last_point.x)**2 +
                         (odom.pose.pose.position.y - self._last_point.y)**2 )
        self._last_point = odom.pose.pose.position
        if distance > 0.2: # some super fast robot!?!
            rospy.logwarn("Odometry jump ignored, suggested robot is super fast?")
            return
        self._current_session.distance += distance
        self._current_session.duration =  odom.header.stamp -  self._local_start_time
        
        # Update mongodb stored session
        try:
            self._msg_store.update(self._current_session,
                                   message_query={'name': self._current_session.name,})
        except rospy.ServiceException, e:
            rospy.logwarn("Marathon reporter could not update the messagestore. Maybe the "
                          "mongodb_store is no longer running?")
        
    def _shutdown_cb(self):
        self._reporting = False
        self._timout.cancel()
        self._timout.join()
        rospy.loginfo("Run completed. Distance travelled=%fm. Duration=%d seconds" %
                      (self._current_session.distance,
                       self._current_session.duration.to_sec()))
        rospy.loginfo("Shutdown!  marking end and submitting...")
        try:
            # The session is a dangler, end it and let the web know
            end = self._web_get(self._current_session.name,
                                self._current_session.distance,
                                self._current_session.duration.to_sec(),
                                end=1)
            if end == "ok":
                self._current_session.ended = True
                self._msg_store.update(self._current_session,
                                       message_query={'name': self._current_session.name,})
                rospy.sleep(2) # would be nicer to know message store had finished clean..
            else:
                rospy.logwarn("Could not mark sesion ended on web, response=%s"%end)
        except REQUESTS_EXCEPTIONS,  e:
            rospy.logwarn("End reporting skipped due to connection error. "
                          "Will finalise report at next run startup.")

    def _die(self):
        rospy.logfatal("!!! !!! THIS RUN WILL NOT BE COUNTED !!! !!!")
        sys.exit(1)
    
if __name__ == '__main__':
    ''' Main Program '''
    reporter = MarathonReporter()
    rospy.spin()
