import math
import rospy

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Path

def clamp(value, lower, upper):
    return min(max(lower, value), upper)


class DoorUtils(object):
    def __init__(self, max_trans_vel, max_rot_vel, base_radius, getting_further_counter_threshold, distance_to_success):
        self.max_trans_vel=max_trans_vel
        self.max_rot_vel=max_rot_vel
        self.base_radius=base_radius
        self.getting_further_counter_threshold=getting_further_counter_threshold #limit of number of consecutive  poses getting further away from the goal to output with 'aborted'
        self.distance_to_success=distance_to_success #once the robot is less than this value from the goal, it stops with success
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
    
    def pose_cb(self, msg): 
        self.new_pose_msg=True
        self.pose_x=msg.position.x
        self.pose_y=msg.position.y
        q0 = msg.orientation.x
        q1 =  msg.orientation.y
        q2 =  msg.orientation.z
        q3 =  msg.orientation.w
        self.current_angle = math.atan2(2.*(q0*q1 + q2*q3), 1. - 2.*(q1**2 + q2**2))

    def scan_cb (self, msg):
        self.new_scan_msg=True
        self.angle_min=msg.angle_min
        self.angle_max=msg.angle_max
        self.angle_increment=msg.angle_increment
        self.range_min=msg.range_min
        self.range_max=msg.range_max
        self.ranges=msg.ranges
    

    def calculate_angle_diff(self, r_x, r_y):
        angle_diff=math.atan2(r_y,r_x)-self.current_angle
        while (angle_diff >= math.pi):
            angle_diff-= 2*math.pi
        while (angle_diff < -math.pi):
            angle_diff += 2*math.pi
        return angle_diff
    
    def rotate_towards_pose(self, target_pose):
        robot_pose_sub = rospy.Subscriber("/robot_pose", Pose, self.pose_cb)
        
        base_cmd=Twist()
        rospy.loginfo("Rotating towards x=" + str(target_pose.position.x) + ", y=" + str(target_pose.position.y));
        self.new_pose_msg=False
        while (not self.new_pose_msg):
            rospy.sleep(0.05)
        r_x = target_pose.position.x-self.pose_x
        r_y = target_pose.position.y-self.pose_y
        angle_diff=self.calculate_angle_diff(r_x,r_y)
        while math.fabs(angle_diff)>0.1:
            if self.new_pose_msg:
                self.new_pose_msg=False
                while (not self.new_pose_msg):
                    rospy.sleep(0.05)
                r_x = target_pose.position.x-self.pose_x
                r_y = target_pose.position.y-self.pose_y
                angle_diff=self.calculate_angle_diff(r_x,r_y)               
                base_cmd.angular.z = angle_diff*0.5;
  
                self.publish_cmd(base_cmd)
        base_cmd.angular.z = 0
  
        self.publish_cmd(base_cmd)

        
    def check_door(self, target_pose): #assumes robot is facing the door
        robot_pose_sub = rospy.Subscriber("/robot_pose", Pose, self.pose_cb)
        scan_sub = rospy.Subscriber("/scan", LaserScan, self.scan_cb)
        self.new_pose_msg=False
        self.new_scan_msg=False
        while (not self.new_pose_msg) or (not self.new_scan_msg):
            rospy.sleep(0.05)
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
            if (angle> -0.26 and angle < 0.26):                            
                if (x < dist_to_goal):
                    closed_door_counter=closed_door_counter+1
                else:
                    open_door_counter=open_door_counter+1
        rospy.loginfo("Front laser door check results. closed_door_counter=" + str(closed_door_counter) + " , open_door_counter=" + str(open_door_counter))                        
        #log result in mongo
        return (open_door_counter>closed_door_counter)
    
    def pass_door(self, target_pose): #assumes robot is facing the door and the door is open
        robot_pose_sub = rospy.Subscriber("/robot_pose", Pose, self.pose_cb)
        scan_sub = rospy.Subscriber("/scan", LaserScan, self.scan_cb)
        base_cmd=Twist()
        self.new_pose_msg=False
        self.new_scan_msg=False
        while (not self.new_pose_msg) or (not self.new_scan_msg):
            rospy.sleep(0.05)
        r_x = target_pose.position.x-self.pose_x
        r_y = target_pose.position.y-self.pose_y
        dist_to_goal=math.sqrt(r_x*r_x+r_y*r_y)
        getting_further_counter=0
        while True:
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
                base_cmd.angular.z =0.0
                base_cmd.linear.x = 0.0
                # doing it once wasn't doing much
                for i in range(5):
                    self.publish_cmd(base_cmd)
                return True
            if (prev_dist_to_goal < dist_to_goal):
                getting_further_counter=getting_further_counter+1
                rospy.loginfo("Getting further away from goal")
            else:
                getting_further_counter=0
                rospy.loginfo("Getting closer to goal")
            if getting_further_counter > self.getting_further_counter_threshold:
                rospy.loginfo("Getting too far from the goal. Door pass failure.")
                base_cmd.angular.z = 0.0
                base_cmd.linear.x = 0.0
                # doing it once wasn't doing much
                for i in range(5):
                    self.publish_cmd(base_cmd)
                return False
            left_minim=100
            right_minim = 100
            left_min_angle=None
            right_min_angle=None
            num_ranges=len(self.ranges)
            for i in range(0,num_ranges):
                angle = self.angle_min+i*self.angle_increment
                d = self.ranges[i]
                x = d*math.cos(angle)
                y = d*math.sin(angle)
                if (x < 0.55):
                    if angle>-math.pi/2 and angle<-math.pi/6:
                        if (right_minim>-y):
                            right_min_angle=angle
                            right_minim = -y
                    elif angle>math.pi/6 and angle<math.pi/2 :
                        if (left_minim>y):
                            left_min_angle=angle
                            left_minim = y
            left_minim-=self.base_radius
            right_minim-=self.base_radius
            rospy.loginfo("left_minim=" + str(left_minim) + " , right_minim=" + str(right_minim))
            base_cmd.linear.x = max(3*min(right_minim,left_minim),0)
            base_cmd.angular.z = 2*(left_minim-right_minim)
            if (left_minim > 0.1 and right_minim >0.1):
                base_cmd.angular.z =0
                base_cmd.linear.x = self.max_trans_vel
            rospy.loginfo("Publishing to cmd_vel: base_cmd.linear.x=" + str(base_cmd.linear.x) + " base_cmd.angular.z=" + str(base_cmd.angular.z))              
            self.publish_cmd(base_cmd)

    def publish_cmd(self, cmd):
        new_cmd = Twist()
        new_cmd.linear.x = clamp(cmd.linear.x, -self.max_trans_vel, self.max_trans_vel)
        new_cmd.angular.z = clamp(cmd.angular.z, -self.max_rot_vel, self.max_rot_vel)
        cmd = new_cmd
        rospy.loginfo("Publishing to cmd_vel: base_cmd.linear.x=" + str(cmd.linear.x) + " cmd.angular.z=" + str(cmd.angular.z))
        self.cmd_vel_pub.publish(cmd)
        
    def set_params(self,
                  max_trans_vel=None,
                  max_rot_vel=None,
                  base_radius=None,
                  getting_further_counter_threshold=None,
                  distance_to_success=None):
        if max_trans_vel is not None:
            self.max_trans_vel=max_trans_vel
        if max_rot_vel is not None:
            self.max_rot_vel=max_rot_vel
        if base_radius is not None:
            self.base_radius=base_radius
        if getting_further_counter_threshold is not None:
            self.getting_further_counter_threshold=getting_further_counter_threshold
        if distance_to_success is not None:
            self.distance_to_success=distance_to_success
        
        
        