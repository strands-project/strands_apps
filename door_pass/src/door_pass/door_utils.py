import math
import rospy

from robot_talk.proxy import RobotTalkProxy

from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Path
from door_pass.msg import DoorCheckStat, DoorWaitStat
from mongodb_store.message_store import MessageStoreProxy
from actionlib import SimpleActionClient
from mary_tts.msg import maryttsAction, maryttsGoal
import strands_webserver.client_utils as cu
from strands_navigation_msgs.srv import LocalisePose



def clamp(value, lower, upper):
    return min(max(lower, value), upper)


class DoorUtils(object):
    def __init__(self,
                 max_trans_vel,
                 max_rot_vel,
                 vel_scale_factor,
                 base_radius,
                 getting_further_counter_threshold,
                 distance_to_success,
                 n_closed_door):
        self.max_trans_vel=max_trans_vel
        self.max_rot_vel=max_rot_vel
        self.vel_scale_factor=vel_scale_factor
        self.base_radius=base_radius
        self.getting_further_counter_threshold=getting_further_counter_threshold #limit of number of consecutive  poses getting further away from the goal to output with 'aborted'
        self.distance_to_success=distance_to_success #once the robot is less than this value from the goal, it stops with success
        self.n_closed_door=n_closed_door #number of laser readings less than the distance to goal in order to classify a door as closed
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.last_twist = None

        self.pose_x=0.0
        self.pose_y=0.0
        self.current_angle=0.0

        self.angle_min=0.0
        self.angle_max=0.0
        self.angle_increment=0.0
        self.range_min=0.0
        self.range_max=0.0
        self.ranges=[]

        self.new_pose_msg = False
        self.new_scan_msg = False
        
        self.is_active=False
        self.get_target_wp_srv=rospy.ServiceProxy("/topological_localisation/localise_pose", LocalisePose)
        self.mongo_logger=message_proxy=MessageStoreProxy(collection='door_stats')
        self.speaker = SimpleActionClient('/speak', maryttsAction)   
        self.just_spoken=False
        self.wait_frequency=0.1
        self.wait_elapsed=0.0
        


        rospy.loginfo("help via screen got webserver services")
        self.display_no = rospy.get_param("~display", 0)

        
        self.talk_proxy = RobotTalkProxy('robot_talk')
        # using topics 'door_pass_ask', 'door_pass_going', and 'door_pass_thanks' 
        
        #self.ask_to_hold=["Please hold the door!",
        #                  "Can you give me a hand?"]
        #self.going_through=["I'm going through now.",
        #                   "Here I go!"]
        #self.thanks=["Great! Thank you for the help.",
        #             "You're so kind!"]
        
        self.screen_message="Please hold the door, I need to get to the other side."
       
        
        
    def activate(self):
        self.is_active=True
        
    def deactivate(self):
        self.is_active=False
    
    def pose_cb(self, msg):
        self.pose_x=msg.position.x
        self.pose_y=msg.position.y
        q0 = msg.orientation.x
        q1 =  msg.orientation.y
        q2 =  msg.orientation.z
        q3 =  msg.orientation.w
        self.current_angle = math.atan2(2.*(q0*q1 + q2*q3), 1. - 2.*(q1**2 + q2**2))
        self.new_pose_msg=True

    def scan_cb (self, msg):
        self.angle_min=msg.angle_min
        self.angle_max=msg.angle_max
        self.angle_increment=msg.angle_increment
        self.range_min=msg.range_min
        self.range_max=msg.range_max
        self.ranges=msg.ranges
        self.new_scan_msg=True

    

    def calculate_angle_diff(self, r_x, r_y):
        print "RY", r_y
        print "RX", r_x
        print "current_angle", self.current_angle
        angle_diff=math.atan2(r_y,r_x)-self.current_angle
        while (angle_diff >= math.pi):
            angle_diff-= 2*math.pi
        while (angle_diff < -math.pi):
            angle_diff += 2*math.pi
        return angle_diff
    
    def rotate_towards_pose(self, target_pose):
        if self.is_active:
            robot_pose_sub = rospy.Subscriber("/robot_pose", Pose, self.pose_cb)
            
            base_cmd=Twist()
            rospy.loginfo("Rotating towards x=" + str(target_pose.position.x) + ", y=" + str(target_pose.position.y));
            self.new_pose_msg=False
            while (not self.new_pose_msg):
                rospy.sleep(0.05)
            r_x = target_pose.position.x-self.pose_x
            r_y = target_pose.position.y-self.pose_y
            angle_diff=self.calculate_angle_diff(r_x,r_y)
            while math.fabs(angle_diff)>0.1 and self.is_active:
                if self.new_pose_msg:
                    self.new_pose_msg=False
                    while (not self.new_pose_msg):
                        rospy.sleep(0.05)
                    r_x = target_pose.position.x-self.pose_x
                    r_y = target_pose.position.y-self.pose_y
                    angle_diff=self.calculate_angle_diff(r_x,r_y)
                    base_cmd.angular.z = angle_diff*0.5;   
                    self.publish_cmd(base_cmd)
            robot_pose_sub.unregister()
            self.stop_robot()

        
    def check_door(self, target_pose=None, log_to_mongo=True): #assumes robot is facing the door
        if self.is_active:
            robot_pose_sub = rospy.Subscriber("/robot_pose", Pose, self.pose_cb)
            scan_sub = rospy.Subscriber("/scan", LaserScan, self.scan_cb)
            self.new_pose_msg=False
            self.new_scan_msg=False
            while (not self.new_pose_msg) or (not self.new_scan_msg):
                rospy.sleep(0.05)
            if target_pose is None:
                dist_to_goal=2            
            else:
                r_x = target_pose.position.x-self.pose_x
                r_y = target_pose.position.y-self.pose_y
                dist_to_goal=math.sqrt(r_x*r_x+r_y*r_y)
            closed_door_counter=0
            open_door_counter=0
            rospy.loginfo("Checking if door is opening using the current distance to goal: "+  str(dist_to_goal))
            for i in range(0, len(self.ranges)):
                angle = self.angle_min+i*self.angle_increment
                d = self.ranges[i]
                x = d*math.cos(angle)+0.07
                if (angle> -0.16 and angle < 0.16):                            
                    if (x < dist_to_goal):
                        closed_door_counter=closed_door_counter+1
                    else:
                        open_door_counter=open_door_counter+1
            rospy.loginfo("Front laser door check results. closed_door_counter=" + str(closed_door_counter) + " , open_door_counter=" + str(open_door_counter))                        
            #log result in mongo
            is_open=closed_door_counter<self.n_closed_door
            if log_to_mongo:
                try:
                    if target_pose is not None:
                        target_waypoint=self.get_target_wp_srv(target_pose).current_node
                    else:
                        target_waypoint='none'
                    waypoint=rospy.wait_for_message("/current_node", String, 5)
                    topological_map_name=rospy.get_param("/topological_map_name", "")
                    self.mongo_logger.insert(DoorCheckStat(topological_map_name=topological_map_name,
                                                        source_waypoint=waypoint.data,
                                                        target_waypoint=target_waypoint,
                                                        is_open=is_open))
                except Exception, e:
                    rospy.logwarn("Error logging door check " + str(e))
            robot_pose_sub.unregister()
            scan_sub.unregister()
            return is_open
        else:
            return False
    
    def wait_door(self, wait_timeout, target_pose=None, log_to_mongo=True, speak=True,consecutive_open_secs=2):
        self.just_spoken=False
        open_time=0
        self.wait_elapsed=0.0
        wait_timer=rospy.Timer(rospy.Duration(self.wait_frequency), self.wait_timer_cb)
        cu.display_content(self.display_no, self.screen_message)
        while self.is_active and self.wait_elapsed < wait_timeout and abs(open_time-consecutive_open_secs)>(self.wait_frequency/2):
            rospy.loginfo("Door wait and pass action server calling check door")
            door_open=self.check_door(target_pose, False)
            if door_open:
                open_time+=self.wait_frequency
            else:
                open_time=0
            if speak and abs(open_time-self.wait_frequency)<0.01 and not self.just_spoken:
                speak_timer=rospy.Timer(rospy.Duration(10), self.speak_timer_cb, oneshot=True)
                self.just_spoken=True
                self.speaker.send_goal(maryttsGoal(text=self.talk_proxy.get_random_text("door_pass_ask")))
            rospy.sleep(rospy.Duration(self.wait_frequency))
        wait_timer.shutdown()
        cu.display_relative_page(self.display_no, 'index.html')
        if not self.is_active:
            return False
      
        print open_time
        print consecutive_open_secs
        opened=(abs(open_time-consecutive_open_secs)<=(self.wait_frequency/2))
        if log_to_mongo:
            try:
                if target_pose is not None:
                    target_waypoint=self.get_target_wp_srv(target_pose).current_node
                else:
                    target_waypoint='none'
                waypoint=rospy.wait_for_message("/current_node", String, 5)
                topological_map_name=rospy.get_param("/topological_map_name", "")
                self.mongo_logger.insert(DoorWaitStat(topological_map_name=topological_map_name,
                                                    source_waypoint=waypoint.data,
                                                    target_waypoint=target_waypoint,
                                                    opened=opened,
                                                    wait_time=self.wait_elapsed))
            except Exception, e:
                rospy.logwarn("Error logging door check " + str(e))
        return opened
        
    
    def speak_timer_cb(self, event):
        self.just_spoken=False
        
    def wait_timer_cb(self, event):
        self.wait_elapsed+=self.wait_frequency
    
    def pass_door(self, target_pose, speech=False): #assumes robot is facing the door and the door is open
        if self.is_active:
            robot_pose_sub = rospy.Subscriber("/robot_pose", Pose, self.pose_cb)
            scan_sub = rospy.Subscriber("/scan", LaserScan, self.scan_cb)
            if speech:
                self.speaker.send_goal(maryttsGoal(text=self.talk_proxy.get_random_text("door_pass_going")))
            base_cmd=Twist()
            self.new_pose_msg=False
            self.new_scan_msg=False
            while (not self.new_pose_msg) or (not self.new_scan_msg):
                rospy.sleep(0.05)
            r_x = target_pose.position.x-self.pose_x
            r_y = target_pose.position.y-self.pose_y
            dist_to_goal=math.sqrt(r_x*r_x+r_y*r_y)
            getting_further_counter=0
            while self.is_active:
                self.new_pose_msg=False
                self.new_scan_msg=False
                prev_dist_to_goal=dist_to_goal
                while (not self.new_pose_msg) or (not self.new_scan_msg):
                    rospy.sleep(0.05)
                r_x = target_pose.position.x-self.pose_x
                r_y = target_pose.position.y-self.pose_y
                dist_to_goal=math.sqrt(r_x*r_x+r_y*r_y)
                if (dist_to_goal<self.distance_to_success):
                    rospy.loginfo("Close enough to goal pose, door pass success.")
                    self.stop_robot()
                    if speech:
                        self.speaker.send_goal(maryttsGoal(text=self.talk_proxy.get_random_text("door_pass_thanks")))
                    robot_pose_sub.unregister()
                    scan_sub.unregister()
                    return True
                if (prev_dist_to_goal < dist_to_goal):
                    getting_further_counter=getting_further_counter+1
                    rospy.loginfo("Getting further away from goal")
                else:
                    getting_further_counter=0
                    rospy.loginfo("Getting closer to goal")
                if getting_further_counter > self.getting_further_counter_threshold:
                    rospy.loginfo("Getting too far from the goal. Door pass failure.")
                    self.stop_robot()
                    robot_pose_sub.unregister()
                    scan_sub.unregister()
                    return False
                left_minim=0.4
                right_minim = 0.4
                front_minim=1
                num_ranges=len(self.ranges)
                for i in range(0,num_ranges):
                    angle = self.angle_min+i*self.angle_increment
                    d = self.ranges[i]
                    x = d*math.cos(angle)
                    y = d*math.sin(angle)
                    if y<0.4:
                        if angle<-math.pi/4:
                            if (right_minim>-y):
                                right_minim = -y
                        elif angle>math.pi/4:
                            if (left_minim>y):
                                left_minim = y
                    if angle>-math.pi/4 and angle<math.pi/4:
                        if front_minim>d:
                            front_minim=d                        
                left_minim-=self.base_radius
                right_minim-=self.base_radius
                front_minim=front_minim-0.2
                rospy.loginfo("left_minim=" + str(left_minim) + " , right_minim=" + str(right_minim) + " , front_minim=" + str(front_minim))
                if front_minim<=0:
                    rospy.loginfo("Too close to an obstacle. Aborting.")
                    self.stop_robot()
                    robot_pose_sub.unregister()
                    scan_sub.unregister()
                    return False
                base_cmd.linear.x = self.vel_scale_factor*self.max_trans_vel*front_minim
                base_cmd.angular.z = self.vel_scale_factor*(left_minim-right_minim)
                if (left_minim > 0.1 and right_minim >0.1):
                    base_cmd.angular.z =0
                    base_cmd.linear.x = self.vel_scale_factor*self.max_trans_vel*front_minim
                rospy.loginfo("Publishing to cmd_vel: base_cmd.linear.x=" + str(base_cmd.linear.x) + " base_cmd.angular.z=" + str(base_cmd.angular.z))              
                self.publish_cmd(base_cmd)
            self.stop_robot()
        robot_pose_sub.unregister()
        scan_sub.unregister()
        return False
        
    def stop_robot(self):
        base_cmd=Twist()
        base_cmd.angular.z = 0.0
        base_cmd.linear.x = 0.0
        for i in range(5):
            self.publish_cmd(base_cmd)

    def publish_cmd(self, cmd):
        if self.is_active:
            new_cmd = Twist()
            new_cmd.linear.x = clamp(cmd.linear.x, -self.max_trans_vel, self.max_trans_vel)
            new_cmd.angular.z = clamp(cmd.angular.z, -self.max_rot_vel, self.max_rot_vel)
            cmd = new_cmd
            rospy.loginfo("Publishing to cmd_vel: base_cmd.linear.x=" + str(cmd.linear.x) + " cmd.angular.z=" + str(cmd.angular.z))
            self.cmd_vel_pub.publish(cmd)
        
    def set_params(self,
                  max_trans_vel=None,
                  max_rot_vel=None,
                  vel_scale_factor=None,
                  base_radius=None,
                  getting_further_counter_threshold=None,
                  distance_to_success=None,
                  n_closed_door=None):
        if max_trans_vel is not None:
            self.max_trans_vel=max_trans_vel
        if max_rot_vel is not None:
            self.max_rot_vel=max_rot_vel
        if vel_scale_factor is not None:
            self.vel_scale_factor=vel_scale_factor
        if base_radius is not None:
            self.base_radius=base_radius
        if getting_further_counter_threshold is not None:
            self.getting_further_counter_threshold=getting_further_counter_threshold
        if distance_to_success is not None:
            self.distance_to_success=distance_to_success
        if n_closed_door is not None:
            self.n_closed_door = n_closed_door
        
        
        
