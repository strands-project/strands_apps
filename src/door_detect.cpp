#include <ros/ros.h>
#include <std_msgs/String.h>
#include <vector>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <strands_navigation_msgs/DoorCheckAction.h>
#include <actionlib/server/simple_action_server.h>
#include "CDoorDetection.h"

typedef actionlib::SimpleActionServer<strands_navigation_msgs::DoorCheckAction> Server;
Server *server;
ros::Subscriber scan_sub;

int measurements = 0;
float maxDistance = 3.0;	
int detections = 0;		
bool debug = true;
float doorWidth = 0;

typedef enum{
	IDLE,
	DETECT,
	STOPPING,
	PREEMPTED,
	SUCCESS,
	FAIL
}EClimbState;

EClimbState state = IDLE;
 
void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
	if (state == DETECT){
		SDoor door = detectDoor(scan_msg->ranges,scan_msg->angle_min,scan_msg->angle_increment,scan_msg->ranges.size(),maxDistance);
		if (door.found){
			doorWidth+=door.doorWidth;
			detections++;
		}
		measurements--;
	}
}

void actionServerCallback(const strands_navigation_msgs::DoorCheckGoalConstPtr& goal, Server* as)
{
	strands_navigation_msgs::DoorCheckResult result;
	state = DETECT;
	measurements = 30;
	detections = 0;
	doorWidth = 0;
	while (state == DETECT){
		if (measurements <= 0){
			// if (detections > 0) state = SUCCESS; else state = FAIL;
			// SUCCESS means that it ran to completion, result communicates open/closed
			state = SUCCESS
		}
		usleep(20000);
	}
	result.open = false;
	if (detections > 0){
		result.width=doorWidth/detections;
		result.open=true;
	}
	if (state == SUCCESS) server->setSucceeded(result);
	if (state == FAIL) server->setAborted(result);
	if (state == PREEMPTED) server->setPreempted(result);
	state = STOPPING;
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "doorDetection");
	ros::NodeHandle n;
	server = new Server(n, "doorDetection", boost::bind(&actionServerCallback, _1, server), false);
	server->start();
	scan_sub = n.subscribe("scan", 100, scanCallback);
	while (ros::ok()){
		if (server->isPreemptRequested() && state != IDLE) state = PREEMPTED;
		if (state == STOPPING)state = IDLE;
		ros::spinOnce();
		usleep(30000);
	}
}

