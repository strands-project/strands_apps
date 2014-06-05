#! /usr/bin/env python

import math
from tf.transformations import euler_from_quaternion

import rospy
import actionlib


from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Pose
from move_base_msgs.msg import MoveBaseAction
from nav_msgs.msg import Path




class DoorPass(object):

    def __init__(self):

        self.action_server=actionlib.SimpleActionServer('doorPassing', MoveBaseAction, execute_cb = self.execute_cb, auto_start=False) 
        self.maxDistance = 3.0                #max range taken into consideration
        self.defaultSpeed = 0.15             #default forward speed of the robot
        self.baseRadius = 0.31;
        self.passCounterLimit = 10              #measurements 
        self.passCounter = 0                    #measurements 

        self.MAX_FURTHER_COUNT=5;
        
        #self.state = 'IDLE' 'TURNING', 'DETECT' ,'DOOR_CLOSED', 'PASS', 'PREEMPTED', 'SUCCESS','FAIL')
        self.state='IDLE'


        self.poseX=0.0
        self.poseY=0.0
        self.currentAngle=0.0


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
        self.poseX=msg.position.x
        self.poseY=msg.position.y
        q0 = msg.orientation.x
        q1 =  msg.orientation.y
        q2 =  msg.orientation.z
        q3 =  msg.orientation.w
        self.currentAngle = math.atan2(2.*(q0*q1 + q2*q3), 1. - 2.*(q1**2 + q2**2))



    def scan_cb (self, msg):
        self.new_scan_msg=True
        self.angle_min=msg.angle_min
        self.angle_max=msg.angle_max
        self.angle_increment=msg.angle_increment
        self.range_min=msg.range_min
        self.range_max=msg.range_max
        self.ranges=msg.ranges
    

    def execute_cb(self, goal):      
            scan_sub = rospy.Subscriber("/scan", LaserScan, self.scan_cb)
            robot_pose = rospy.Subscriber("/robot_pose", Pose, self.pose_cb)     
            cmd_vel=rospy.Publisher("/cmd_vel", Twist)
            path_pub=rospy.Publisher("/move_base/NavfnROS/plan",Path)
            
            self.new_pose_msg=False
            self.new_scan_msg=False
            
            base_cmd = Twist()
            empty_path = Path() #to make sure help is not asked when door is closed
            
            #move_base_msgs::MoveBaseResult result;
            goalX = goal.target_pose.pose.position.x;
            goalY = goal.target_pose.pose.position.y;

            

            #float rX;
            #float rY;
            #float prevDistToGoal;
            distToGoal=0.0
            gettingFurtherCounter=0


            closedDoorFinalAngle=1.550  #hack to handle narrow g4s corridor
            
            #size_t num_ranges;
            #float d;
            closedDoorCounter = 0
            openDoorCounter = 0
            #float angle;
            #float x;
            #float y;
            #float leftMinim,rightMinim;
            #int i;


            

            print("ACTION SERVER CALLBACK JUST STARTED. GOAL is x=%f, y=%f\n", goalX, goalY); 
            print("GOING TO STATE TURNING\n");
            self.state = 'TURNING';
            while (self.state == 'TURNING' or self.state == 'DETECT' or self.state == 'PASS'):
                while((not self.new_pose_msg) or (not self.new_scan_msg)):
                    rospy.sleep(0.05)
             
                
                print("CHECKING PREEMPTED\n");
                if (self.action_server.is_preempt_requested()):
                        print("GOING TO STATE PREEMPTEDD\n")
                        self.state = 'PREEMPTED'
                
                
                self.new_pose_msg=False
                self.new_scan_msg=False
                rX = goalX-self.poseX
                rY = goalY-self.poseY
                prevDistToGoal=distToGoal
                distToGoal=math.sqrt(rX*rX+rY*rY)
                num_ranges = len(self.ranges)
                
                if self.state == 'TURNING':
                        print("IN STATE TURNING\n");
                        base_cmd.linear.x = 0; 
                        self.currentAngle = (math.atan2(rY,rX)-self.currentAngle)
                        while (self.currentAngle >= math.pi):
                            self.currentAngle-= 2*math.pi
                        while (self.currentAngle < -math.pi):
                            self.currentAngle += 2*math.pi
                        base_cmd.angular.z = self.currentAngle*0.5;
                        #maxDistance = fmin(distToGoal,maxDistance);
                        if (math.fabs(self.currentAngle) < 0.1):
                            base_cmd.angular.z = 0;
                            print("FACING GOAL POSE, GOING TO STATE DETECT\n")
                            self.state = 'DETECT'
                                
                        print("publishing to cmd_vel: base_cmd.linear.x = %f, base_cmd.angular.z= %f\n", base_cmd.linear.x, base_cmd.angular.z)
                        cmd_vel.publish(base_cmd)
                
                elif self.state == 'DETECT':
                        print("IN STATE DETECT\n");
                        print("CHECKING OPEN DOOR. DIST TO GOAL: %f\n", distToGoal)
                        for i in range(0, num_ranges):
                            angle = self.angle_min+i*self.angle_increment
                            d = self.ranges[i]
                            x = d*math.cos(angle)+0.07
                            if (angle> -0.26 and angle < 0.26):                            
                                if (x < distToGoal):
                                    closedDoorCounter=closedDoorCounter+1
                                else:
                                    openDoorCounter=openDoorCounter+1
                                
                            
                        
                        print("CLOSED DOOR: %d\n", closedDoorCounter)
                        print("OPEN DOOR: %d\n", openDoorCounter)
                        if (closedDoorCounter > openDoorCounter):
                            print("GOING TO STATE DOOR_CLOSED\n")
                            self.state='DOOR_CLOSED'
                        else:
                            print("GOING TO STATE PASS\n")
                            self.state='PASS'


                        
                elif self.state == 'PASS':
                        print("IN STATE PASS\n");
                        
                        if (prevDistToGoal < distToGoal):
                            gettingFurtherCounter=gettingFurtherCounter+1
                            print("GETTING FURTHER AWAY FROM GOAL\n")
                        else:
                            print("GETTING CLOSER TO GOAL\n")
                            gettingFurtherCounter=0

                        
                        leftMinim=100
                        rightMinim = 100 
                        for i in range(0,num_ranges):
                            angle = self.angle_min+i*self.angle_increment
                            d = self.ranges[i]
                            x = d*math.cos(angle);
                            y = d*math.sin(angle);
                            #print("ANGLE= %f, RANGES = %f, d = %f, x = %f, y = %f, angle_min=%f, angle_increment=%f\n",  angle, ranges[i], d, x, y,angle_min,angle_increment);
                            if (x < 0.55):
                                if (i<num_ranges/2):
                                    if (rightMinim>-y):
                                        rightMinim = -y
                                else:
                                    if (leftMinim>y):
                                        leftMinim = y
                                
                             
                        leftMinim-=self.baseRadius
                        rightMinim-=self.baseRadius
                        print("leftMinim: %f, rightMinin:  %f \n",leftMinim,rightMinim)
                        base_cmd.linear.x = max(min(3*min(rightMinim,leftMinim),self.defaultSpeed),0)
                        base_cmd.angular.z = 2*(leftMinim-rightMinim)
                        print("base_cmd.linear.x = %f, base_cmd.angular.z= %f\n", base_cmd.linear.x, base_cmd.angular.z);
                        if (leftMinim > 0.1 and rightMinim >0.1):
                            base_cmd.angular.z =0
                            base_cmd.linear.x = 0.2
                            print("leftMinim > 0.1 && rightMinim >0.1, updating: base_cmd.linear.x = %f, base_cmd.angular.z= %f\n", base_cmd.linear.x, base_cmd.angular.z)

                        if (distToGoal<0.2):
                            print("close enough to goal pose, going to state SUCCESS\n")
                            self.state='SUCCESS'
                        
                        if (gettingFurtherCounter > self.MAX_FURTHER_COUNT):
                            print("getting further away from goal, going to state FAIL\n")
                            self.state='FAIL'
                        
                        print("publishing to cmd_vel: base_cmd.linear.x = %f, base_cmd.angular.z= %f\n", base_cmd.linear.x, base_cmd.angular.z);
                        cmd_vel.publish(base_cmd);

                        
                        
            
                elif self.state == 'DOOR_CLOSED':
                    print("IN STATE DOOR_CLOSED\n");
                    base_cmd.linear.x = 0 
                    self.currentAngle = (closedDoorFinalAngle-self.currentAngle)
                    while (self.currentAngle >= math.pi):
                        self.currentAngle-= 2*math.pi
                    while (self.currentAngle < -math.pi):
                        self.currentAngle += 2*math.pi
                    print("TURNING TO PRE-DEFINED POSITION BEFORE ABORTING\n")
                    base_cmd.angular.z = self.currentAngle*0.5
                    #maxDistance = fmin(distToGoal,maxDistance);
                    if (math.fabs(self.currentAngle) < 0.1):
                        base_cmd.angular.z = 0
                        path_pub.publish(empty_path) #get global nav failure so no help is asked
                        print("REACHED PREDEFINED ORIENTATION?+> GOING TO STATE FAIL\n")
                        self.state = 'FAIL'
                    
                    print("PUBLISHING TO CMD_VEL TO GET INTO PRE DEFINED ORIENTATION\n")
                    cmd_vel.publish(base_cmd)
            
            
            if self.action_server.is_preempt_requested():
                self.state = 'PREEMPTED'

                
            print("STOPPING\n")
            base_cmd.linear.x = 0
            base_cmd.angular.z = 0
            cmd_vel.publish(base_cmd)
            print("STOPPED\n")
            
            #print("scan_sub.shutdown();\n")
            #scan_sub.unregister()
            #print("robot_pose.shutdown();\n")
            #robot_pose.unregister()
            #print("cmd_vel.shutdown();\n")
            #cmd_vel.unregister()
            #print("path_pub.shutdown();\n")
            #path_pub.unregister()
            #print("DONE\n")
            
            if (self.action_server.is_preempt_requested()):
                self.state = 'PREEMPTED'
            
                        
            if (self.state == 'SUCCESS'):
                self.state='IDLE'
                print("DOOR PASS SUCCEEDED\n")
                self.action_server.set_succeeded()
                return
            
            if (self.state == 'FAIL'):
                self.state='IDLE'
                self.action_server.set_aborted()
                print("DOOR PASS FAILED\n");
                return
            
            if (self.state == 'PREEMPTED'):
                self.state='IDLE'
                self.action_server.set_preempted()
                print("DOOR PASS PREEMPTED\n")
                return
            
    

    
if __name__ == '__main__':
    rospy.init_node("door_pass_node")
    
    passer=DoorPass()
    passer.action_server.start()
    rospy.spin()
        
    
    
    
    

