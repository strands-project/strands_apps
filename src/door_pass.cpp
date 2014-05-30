#include <ros/ros.h>
#include <tf/tf.h>
#include <std_msgs/String.h>
#include <vector>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/server/simple_action_server.h>
#include <nav_msgs/Path.h>
#include "CDoorDetection.h"

typedef actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction> Server;
Server *server;
ros::Subscriber scan_sub;
ros::Subscriber robot_pose;

float maxDistance = 3.0;		//max range taken into consideration
float defaultSpeed = 0.15;		//default forward speed of the robot
float baseRadius = 0.31;
int passCounterLimit = 10;		//measurements 
int passCounter = 0;			//measurements 
int maxMisdetections = 50;		//decides when door not detected
int misdetections = 0;
bool debug = true;

int MAX_CLOSED_DOOR_COUNT=5;

typedef enum{
	IDLE,
	TURNING,
	DETECT,
        DOOR_CLOSED,
	APPROACH,
	ADJUST,
	PASS,
	LEAVE,
	STOPPING,
	PREEMPTED,
	SUCCESS,
	FAIL
}EClimbState;

EClimbState state = IDLE;
ros::Publisher cmd_vel;
ros::Publisher path_pub;
geometry_msgs::Twist base_cmd;
nav_msgs::Path empty_path = nav_msgs::Path(); //to make sure help is not asked when door is closed
float goalX;
float goalY;
float distToGoal;




void poseCallback(const geometry_msgs::Pose::ConstPtr& msg)
{       
        float poseX=msg->position.x;
        float poseY=msg->position.y;
        float rX=goalX-poseX;
        float rY = goalY-poseY;
        distToGoal=sqrt(rX*rX+rY*rY);
        
        float currentAngle = tf::getYaw(msg->orientation);
        float closedDoorFinalAngle=1.550;
       // printf("DIST: %f\n", distToGoal);
	if (state == TURNING){
		printf("IN STATE TURNING\n");
		base_cmd.linear.x = 0; 
		currentAngle = (atan2(rY,rX)-currentAngle);
		while (currentAngle >= M_PI) currentAngle-= 2*M_PI;
		while (currentAngle < -M_PI) currentAngle += 2*M_PI;
		base_cmd.angular.z = currentAngle*0.5;
		maxDistance = fmin(distToGoal,maxDistance);
		if (fabs(currentAngle) < 0.1){
			base_cmd.angular.z = 0;
                        printf("FACING GOAL POSE, GOING TO STATE DETECT\n");
			state = DETECT;
		}
		printf("publishing to cmd_vel: base_cmd.linear.x = %f, base_cmd.angular.z= %f\n", base_cmd.linear.x, base_cmd.angular.z);
		cmd_vel.publish(base_cmd);
	}
	if (state == DOOR_CLOSED){
                printf("IN STATE DOOR_CLOSED\n");
                base_cmd.linear.x = 0; 
                currentAngle = (closedDoorFinalAngle-currentAngle);
                while (currentAngle >= M_PI) currentAngle-= 2*M_PI;
                while (currentAngle < -M_PI) currentAngle += 2*M_PI;
                printf("TURNING TO PRE-DEFINED POSITION BEFORE ABORTING\n");
                base_cmd.angular.z = currentAngle*0.5;
                maxDistance = fmin(distToGoal,maxDistance);
                if (fabs(currentAngle) < 0.1){
                        base_cmd.angular.z = 0;
                        path_pub.publish(empty_path);
                        printf("REACHED PREDEFINED ORIENTATION?+> GOING TO STATE FAIL\n");
                        state = FAIL;
                }
                printf("PUBLISHING TO CMD_VEL TO GET INTO PRE DEFINED ORIENTATION\n");
                cmd_vel.publish(base_cmd);
        }
            
            
}

void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_msg) {    
                float d;
                float angle_min;
                float angle_increment;
                size_t num_ranges = scan_msg->ranges.size();
                int closedDoorCounter = 0;
                int openDoorCounter = 0;
                float angle;
                float x;
                float y;
                float leftMinim,rightMinim;
                int i;
                
                if (state == DETECT){
                    printf("IN STATE DETECT\n");
                    angle_min = scan_msg->angle_min;
                    angle_increment =scan_msg->angle_increment;
                    printf("CHECKING OPEN DOOR. DIST TO GOAL: %f\n", distToGoal);
                    for (i = 0; i <= num_ranges; i++) {
                        angle = angle_min+i*angle_increment;
                        d = scan_msg->ranges[i];
                        x = d*cos(angle)+0.07;
                        if (angle> -0.26 && angle < 0.26) {                            
                            if (x < distToGoal) {
                                closedDoorCounter++;
                            } else {
                                openDoorCounter++;
                            }
                        }
                    }
                    printf("CLOSED DOOR: %d\n", closedDoorCounter);
                    printf("OPEN DOOR: %d\n", openDoorCounter);
                    if (closedDoorCounter > openDoorCounter){
                        printf("GOING TO STATE DOOR_CLOSED\n");
                        state=DOOR_CLOSED;
                    } else {
                        printf("GOING TO STATE PASS\n");
			state=PASS;
                    }
                }
		if (state == PASS){
			printf("IN STATE PASS\n");
			leftMinim=rightMinim = 100;


			for (i = 0;i<scan_msg->ranges.size();i++){
				angle = scan_msg->angle_min+i*scan_msg->angle_increment;
				d = fmin(scan_msg->ranges[i],maxDistance);
				x = d*cos(angle);
				y = d*sin(angle);
				if (x < 0.55){
					if (i<scan_msg->ranges.size()/2){
						if (rightMinim>-y) rightMinim = -y;
					}else{
						if (leftMinim>y) leftMinim = y;
					}
				} 
			}
			leftMinim-=baseRadius;
			rightMinim-=baseRadius;
			printf("leftMinim: %f, rightMinin:  %f \n",leftMinim,rightMinim);
			base_cmd.linear.x = fmax(fmin(3*fmin(rightMinim,leftMinim),defaultSpeed),0);
			base_cmd.angular.z = 2*(leftMinim-rightMinim);
                        printf("base_cmd.linear.x = %f, base_cmd.angular.z= %f\n", base_cmd.linear.x, base_cmd.angular.z);
			if (leftMinim > 0.1 && rightMinim >0.1){
				base_cmd.angular.z =0;
				base_cmd.linear.x = 0.2;
                                printf("leftMinim > 0.1 && rightMinim >0.1, updating: base_cmd.linear.x = %f, base_cmd.angular.z= %f\n", base_cmd.linear.x, base_cmd.angular.z);
			}
			//if (passCounter > passCounterLimit) state = LEAVE;
			if (distToGoal<0.2){
                            printf("close enough to goal pose, going to state LEAVE\n");
                            state=LEAVE;
                        }
                        if (distToGoal > 1.5){
                            printf("too far from goal, going to state FAIL\n");
                            state=FAIL;
                        }
                        printf("publishing to cmd_vel: base_cmd.linear.x = %f, base_cmd.angular.z= %f\n", base_cmd.linear.x, base_cmd.angular.z);
                        cmd_vel.publish(base_cmd);
		}
//	}
}

void actionServerCallback(const move_base_msgs::MoveBaseGoalConstPtr& goal, Server* as)
{
	move_base_msgs::MoveBaseResult result;
	goalX = goal->target_pose.pose.position.x;
	goalY = goal->target_pose.pose.position.y;
        printf("ACTION SERVER CALLBACK JUST STARTED. GOAL is x=%f, y=%f\n", goalX, goalY); 
	printf("GOING TO STATE TURNING\n");
	state = TURNING;
	while (state == TURNING || state == DETECT || state == DOOR_CLOSED||  state == PASS || state == LEAVE){
		if (misdetections > maxMisdetections || state == LEAVE){
			if (state == LEAVE) state = SUCCESS; else state = FAIL;
		}
		ros::Duration(0.2).sleep();
	}
	if (state == SUCCESS) server->setSucceeded(result);
	if (state == FAIL) server->setAborted(result);
	if (state == PREEMPTED) server->setPreempted(result);
        printf("ACTION FINISHED, GOING TO STATE STOPPING\n");
	state = STOPPING;
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "door_pass_node");
	ros::NodeHandle n;
	cmd_vel = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
        path_pub=n.advertise<nav_msgs::Path>("/move_base/NavfnROS/plan",1);
	robot_pose = n.subscribe("/robot_pose", 1000, poseCallback);
	server = new Server(n, "doorPassing", boost::bind(&actionServerCallback, _1, server), false);
	server->start();
	scan_sub = n.subscribe("scan", 100, scanCallback);
	while (ros::ok()){
		if (server->isPreemptRequested() && state != IDLE) state = PREEMPTED; 
		if (state == STOPPING)
		{       
                        printf("IN STATE STOPPING\n");
			base_cmd.linear.x = base_cmd.angular.z = 0;
			cmd_vel.publish(base_cmd);
                        printf("PUBLISHED 0 VEL COMMANDS, GOING TO STATE IDLE\n");
			state = IDLE;
		} 
		ros::spinOnce();
		ros::Duration(0.3).sleep();
	}
}

