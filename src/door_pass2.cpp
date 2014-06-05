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


class DoorPass {
protected:

    ros::NodeHandle n;
    // NodeHandle instance must be created before this line. Otherwise strange error may occur.
    actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction> as; 
    static const float maxDistance = 3.0;                //max range taken into consideration
    static const float defaultSpeed = 0.15;              //default forward speed of the robot
    static const float baseRadius = 0.31;
    static const int passCounterLimit = 10;              //measurements 
    static const int passCounter = 0;                    //measurements 
    static const int maxMisdetections = 50;              //decides when door not detected
    static const int misdetections = 0;
    static const bool debug = true;

    static const int MAX_FURTHER_COUNT=5;

    typedef enum{
            TURNING,
            DETECT,
            DOOR_CLOSED,
            PASS,
            PREEMPTED,
            SUCCESS,
            FAIL
    }door_pass_state;


    door_pass_state state;

    float poseX;
    float poseY;
    float currentAngle;


    float angle_min;
    float angle_max;
    float angle_increment;
    float range_min;
    float range_max;
    std::vector<float> ranges;

    bool new_pose_msg;
    bool new_scan_msg;

public:
    
    DoorPass(void) :
        as(n, "doorPassing", boost::bind(&DoorPass::actionServerCallback, this, _1), false)
    {
        as.start();
    }
    
    void poseCallback(const geometry_msgs::Pose::ConstPtr& msg) {
        new_pose_msg=true;
        poseX=msg->position.x;
        poseY=msg->position.y;
        currentAngle= tf::getYaw(msg->orientation); 
    }

    void scanCallback (const sensor_msgs::LaserScan::ConstPtr& msg) {
        new_scan_msg=true;
        angle_min=msg->angle_min;
        angle_max=msg->angle_max;
        angle_increment=msg->angle_increment;
        range_min=msg->range_min;
        range_max=msg->range_max;
        ranges=msg->ranges;
    }

    void actionServerCallback(const move_base_msgs::MoveBaseGoalConstPtr &goal)
    {       
            ros::Subscriber scan_sub = n.subscribe("/scan", 1, &DoorPass::scanCallback,this);
            ros::Subscriber robot_pose = n.subscribe("/robot_pose", 1, &DoorPass::poseCallback, this);        
            ros::Publisher cmd_vel=n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
            ros::Publisher path_pub=n.advertise<nav_msgs::Path>("/move_base/NavfnROS/plan",1);
            
            new_pose_msg=false;
            new_scan_msg=false;
            
            geometry_msgs::Twist base_cmd = geometry_msgs::Twist();
            nav_msgs::Path empty_path = nav_msgs::Path(); //to make sure help is not asked when door is closed
            
            move_base_msgs::MoveBaseResult result;
            float goalX = goal->target_pose.pose.position.x;
            float goalY = goal->target_pose.pose.position.y;

            

            float rX;
            float rY;
            float prevDistToGoal;
            float distToGoal=0;
            int gettingFurtherCounter=0;


            float closedDoorFinalAngle=1.550; //hack to handle narrow g4s corridor
            
            size_t num_ranges;
            float d;
            int closedDoorCounter = 0;
            int openDoorCounter = 0;
            float angle;
            float x;
            float y;
            float leftMinim,rightMinim;
            int i;


            

            printf("ACTION SERVER CALLBACK JUST STARTED. GOAL is x=%f, y=%f\n", goalX, goalY); 
            printf("GOING TO STATE TURNING\n");
            state = TURNING;
            while (state == TURNING || state == DETECT || state == PASS) {
                while((! new_pose_msg) || (! new_scan_msg)) {
                    ros::Duration(0.05).sleep();
                }
                
                printf("CHECKING PREEMPTED\n");
                if (as.isPreemptRequested()) {
                        printf("GOING TO STATE PREEMPTEDD\n");
                        state = PREEMPTED;
                }
                printf("CHECKED\n");
                new_pose_msg=false;
                new_scan_msg=false;
                rX = goalX-poseX;
                rY = goalY-poseY;
                prevDistToGoal=distToGoal;
                distToGoal=sqrt(rX*rX+rY*rY);
                num_ranges = ranges.size();
                
                switch(state) {
                    case TURNING:
                        printf("IN STATE TURNING\n");
                        base_cmd.linear.x = 0; 
                        currentAngle = (atan2(rY,rX)-currentAngle);
                        while (currentAngle >= M_PI) currentAngle-= 2*M_PI;
                        while (currentAngle < -M_PI) currentAngle += 2*M_PI;
                        base_cmd.angular.z = currentAngle*0.5;
                        //maxDistance = fmin(distToGoal,maxDistance);
                        if (fabs(currentAngle) < 0.1){
                                base_cmd.angular.z = 0;
                                printf("FACING GOAL POSE, GOING TO STATE DETECT\n");
                                state = DETECT;
                        }
                        printf("publishing to cmd_vel: base_cmd.linear.x = %f, base_cmd.angular.z= %f\n", base_cmd.linear.x, base_cmd.angular.z);
                        cmd_vel.publish(base_cmd);
                        break;
                
                    case DETECT:
                        printf("IN STATE DETECT\n");
                        printf("CHECKING OPEN DOOR. DIST TO GOAL: %f\n", distToGoal);
                        for (i = 0; i <= num_ranges; i++) {
                            angle = angle_min+i*angle_increment;
                            d = ranges[i];
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
                        break;
                        
                    case PASS:
                        printf("IN STATE PASS\n");
                        
                        if (prevDistToGoal < distToGoal) {
                            gettingFurtherCounter++;
                            printf("GETTING FURTHER AWAY FROM GOAL\n");
                        } else {
                            printf("GETTING CLOSER TO GOAL\n");
                            gettingFurtherCounter=0;
                        }
                        
                        leftMinim=rightMinim = 100; 
                        for (i = 0;i<num_ranges;i++){
                            angle = angle_min+i*angle_increment;
                            d = fmin(ranges[i],maxDistance);
                            x = d*cos(angle);
                            y = d*sin(angle);
                            //printf("ANGLE= %f, RANGES = %f, d = %f, x = %f, y = %f, angle_min=%f, angle_increment=%f\n",  angle, ranges[i], d, x, y,angle_min,angle_increment);
                            if (x < 0.55){
                                if (i<num_ranges/2){
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
                        if (distToGoal<0.2){
                            printf("close enough to goal pose, going to state SUCCESS\n");
                            state=SUCCESS;
                        }
                        if (gettingFurtherCounter > MAX_FURTHER_COUNT){
                            printf("getting further away from goal, going to state FAIL\n");
                            state=FAIL;
                        }
                        printf("publishing to cmd_vel: base_cmd.linear.x = %f, base_cmd.angular.z= %f\n", base_cmd.linear.x, base_cmd.angular.z);
                        cmd_vel.publish(base_cmd);
                        break;

                }
            }
            
            if (state == DOOR_CLOSED){
                printf("IN STATE DOOR_CLOSED\n");
                base_cmd.linear.x = 0; 
                currentAngle = (closedDoorFinalAngle-currentAngle);
                while (currentAngle >= M_PI) currentAngle-= 2*M_PI;
                while (currentAngle < -M_PI) currentAngle += 2*M_PI;
                printf("TURNING TO PRE-DEFINED POSITION BEFORE ABORTING\n");
                base_cmd.angular.z = currentAngle*0.5;
                //maxDistance = fmin(distToGoal,maxDistance);
                if (fabs(currentAngle) < 0.1){
                    base_cmd.angular.z = 0;
                    path_pub.publish(empty_path);
                    printf("REACHED PREDEFINED ORIENTATION?+> GOING TO STATE FAIL\n");
                    state = FAIL;
                }
                printf("PUBLISHING TO CMD_VEL TO GET INTO PRE DEFINED ORIENTATION\n");
                cmd_vel.publish(base_cmd);
            }
            
            if (as.isPreemptRequested()) {
                state = PREEMPTED;
            }
                
            printf("STOPPING\n");
            base_cmd.linear.x = 0;
            base_cmd.angular.z = 0;
            cmd_vel.publish(base_cmd);
            printf("STOPPED\n");
            
            printf("scan_sub.shutdown();\n");
            scan_sub.shutdown();
            printf("robot_pose.shutdown();\n");
            robot_pose.shutdown();
            printf("cmd_vel.shutdown();\n");
            cmd_vel.shutdown();
            printf("path_pub.shutdown();\n");
            path_pub.shutdown();
            printf("DONE\n");
            
            if (as.isPreemptRequested()) {
                state = PREEMPTED;
            }
                        
            if (state == SUCCESS) {
                printf("DOOR PASS SUCCEEDED\n");
                as.setSucceeded(result);
                return;
            }
            if (state == FAIL) {
                as.setAborted(result);
                printf("DOOR PASS FAILED\n");
                return;
            }
            if (state == PREEMPTED) {
                as.setPreempted(result);
                printf("DOOR PASS PREEMPTED\n");
                return;
            }
            printf("EXITING\n");
    }
};

int main(int argc, char** argv)
{
        ros::init(argc, argv, "door_pass_node");
        DoorPass passer();
        ros::spin();
        
        return 0;
/*
        while (ros::ok()){
                if (server->isPreemptRequested()) {
                    state = PREEMPTED;
                }
                ros::spinOnce();
                ros::Duration(0.3).sleep();
                printf("ROSLOOP COMPLETE\n");
        }*/
}

