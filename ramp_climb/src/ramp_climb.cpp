#include <ros/ros.h>
#include <tf/tf.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <ramp_climb/RampClimbingAction.h>
#include <actionlib/server/simple_action_server.h>

typedef actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction> Server;
ros::Subscriber robot_pose;
float goalX;
float goalY;

Server *server;
float tolerance = 0.025;
float angleTolerance = 0.10;
float distanceTolerance = 0.05;
float distance = 0.80;
ros::Subscriber scan_sub;
int minPoints =0;

typedef enum{
	IDLE,
	TURNING,
	PROGRESS,
	FINAL,
	STOPPING,
	PREEMPTED,
	SUCCESS,
	FAIL
}EClimbState;

int misdetections = 0;
EClimbState state = IDLE;
ros::Publisher cmd_vel;
geometry_msgs::Twist base_cmd;
float fwSpeed = 0;

void poseCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
	if (state == TURNING){
		float rX,rY;
		rX=rY=0;
		rX = goalX-msg->position.x;
		rY = goalY-msg->position.y;
		float currentAngle = tf::getYaw(msg->orientation);
		base_cmd.linear.x = 0; 
		currentAngle = (atan2(rY,rX)-currentAngle);
		while (currentAngle >= M_PI) currentAngle-= 2*M_PI;
		while (currentAngle < -M_PI) currentAngle += 2*M_PI;
		base_cmd.angular.z = currentAngle*0.5;
		if (fabs(currentAngle) < 0.1){
			base_cmd.angular.z = 0;
			state = PROGRESS;
		} 
		cmd_vel.publish(base_cmd);
	}
}
     
void precise(float *ai,float *bi,float *x,float *y,int num_ranges)
{
	float a = *ai;
	float b = *bi;
	float sxx,sxy,sx,sy;
	int n = 0;
	sxx=sxy=sx=sy=0;
	for (int j = 0; j <= num_ranges; j++){
		if (fabs(a*x[j]-y[j]+b)<tolerance){
			sx += x[j];
			sy += y[j];
			sxx += x[j]*x[j];
			sxy += x[j]*y[j];
			n++;
		}
	}
	if ((n*sxx-sx*sx) != 0 && n > 0){
		a = (n*sxy-sx*sy)/(n*sxx-sx*sx);
		b = (sy-a*sx)/n;
		*ai=a;
		*bi=b;
	}
}

void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
	if (state == PROGRESS || state == FINAL){
		size_t num_ranges = scan_msg->ranges.size();
		float x[num_ranges];
		float y[num_ranges];
		bool m[num_ranges];
		float angle;
		misdetections++;
		for (int i = 0; i <= num_ranges; i++){
			angle = scan_msg->angle_min+i*scan_msg->angle_increment;
			x[i] = scan_msg->ranges[i]*cos(angle);
			y[i] = scan_msg->ranges[i]*sin(angle);
			m[i] = true;
		}
		int eval = 0;
		int max_iterations = 1000;
		int numHypotheses = 10;
		float a,b;
		float maxA[numHypotheses];
		float maxB[numHypotheses];
		int maxEval[numHypotheses];
		for (int h = 0; h <= numHypotheses; h++){
			maxEval[h] = 0;
			for (int i = 0; i <= max_iterations; i++)
			{
				int aIndex = rand()%num_ranges; 
				int bIndex = rand()%num_ranges;
				while (bIndex == aIndex) bIndex = rand()%num_ranges;
				a = (y[bIndex]-y[aIndex])/(x[bIndex]-x[aIndex]);
				b = y[bIndex]-a*x[bIndex];
				eval = 0;
				for (int j = 0; j <= num_ranges; j++){
					if (fabs(a*x[j]-y[j]+b)<tolerance && m[j]) eval++;
				}
				if (maxEval[h] < eval){
					maxEval[h]=eval;
					maxA[h]=a;
					maxB[h]=b;
				}
			}
			for (int j = 0; j <= num_ranges; j++){
				if (fabs(maxA[h]*x[j]-y[j]+maxB[h])<tolerance && m[j]) m[j] = false;
			}
		}
		eval = 0;
		base_cmd.linear.x = base_cmd.angular.z = 0;
		for (int h1 = 0; h1 <= numHypotheses; h1++){
			//		fprintf(stdout,"Ramp hypothesis: %i %f %f %i ",h1,maxA[h1],maxB[h1],maxEval[h1]);
			precise(&maxA[h1],&maxB[h1],x,y,num_ranges);	
			//		fprintf(stdout,"correction: %i %f %f %i\n",h1,maxA[h1],maxB[h1],maxEval[h1]);
		}
		int b1 = -1;
		int b2 = -1;
		eval = 0; 
		float realDist,realAngle,displacement;
		for (int h1 = 0; h1 <= numHypotheses; h1++){
			for (int h2 = h1+1; h2 <= numHypotheses; h2++){
				if (fabs(maxA[h1]-maxA[h2]) < angleTolerance && maxEval[h1] > minPoints && maxEval[h2] > minPoints ){
					realAngle = (maxA[h1]+maxA[h2])/2.0;
					realDist = fabs(maxB[h1]-maxB[h2])*cos(atan(realAngle));
					fprintf(stdout,"Ramp hypothesis: %i %i %f %f %i %i\n",h1,h2,realDist,fabs(maxA[h1]-maxA[h2]),maxEval[h1],maxEval[h2]);
					if (fabs(realDist-distance)<distanceTolerance){
						if (maxEval[h1]+maxEval[h2] > eval){
							eval = maxEval[h1] + maxEval[h2];
							b1 = h1;
							b2 = h2;
						}
					}
				}
			}
		}
		if (b1 >= 0 && b2 >=0){
			displacement = (maxB[b1]+maxB[b2])/2.0;
			realAngle = (maxA[b1]+maxA[b2])/2.0;
			fprintf(stdout,"Ramp found: %i %i %f %f %i %i\n",b1,b2,realDist,fabs(maxA[b1]-maxA[b2]),maxEval[b1],maxEval[b2]);
			if (maxEval[b1]+maxEval[b2]> 360){
				state = FINAL;
				minPoints = 50;
				fwSpeed = fmax(fwSpeed,fmin(maxEval[b1],maxEval[b2])*0.0015); 
			}
			eval = maxEval[b1]+maxEval[b2];
			base_cmd.angular.z = realAngle+3*displacement;
			base_cmd.linear.x = fmax(fwSpeed,fmin(maxEval[b1],maxEval[b2])*0.0015);
			//if (fabs(displacement)>0.1 ) base_cmd.linear.x -= fabs(base_cmd.angular.z);
			if (base_cmd.linear.x < 0) base_cmd.linear.x = 0;
			float spdLimit = 0.8;
			if (base_cmd.angular.z > spdLimit) base_cmd.angular.z = spdLimit ;
			if (base_cmd.angular.z < -spdLimit) base_cmd.angular.z = -spdLimit ;
			cmd_vel.publish(base_cmd);
			misdetections=misdetections/2;
		}else if (state == FINAL){
			base_cmd.angular.z = 0; 
			base_cmd.linear.x = fwSpeed; 
			cmd_vel.publish(base_cmd);
		}else{
			base_cmd.angular.z = 0; 
			base_cmd.linear.x = 0; 
			cmd_vel.publish(base_cmd);
		}
	}
}

void actionServerCallback(const move_base_msgs::MoveBaseGoalConstPtr& goal, Server* as)
{
	move_base_msgs::MoveBaseResult result;
//	ramp_climb::RampClimbingResult result;
	fwSpeed = 0.0;
	misdetections = 0;
	minPoints = 25;
	state = TURNING;
	goalX = goal->target_pose.pose.position.x;
	goalY = goal->target_pose.pose.position.y;
	if (goalX == 0 && goalY == 0) state = PROGRESS;
	while (state == TURNING || state == PROGRESS || state == FINAL){
		if (misdetections > 20){
			if (state == FINAL) state = SUCCESS; else state = FAIL;
		}
		usleep(15000);
	}
	if (state == SUCCESS) server->setSucceeded(result);
	if (state == FAIL) server->setAborted(result);
	if (state == PREEMPTED) server->setPreempted(result);
	state = STOPPING;
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "rampClimbing");
	ros::NodeHandle n;
	fwSpeed = 0.0;
	cmd_vel = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	server = new Server(n, "rampClimbingServer", boost::bind(&actionServerCallback, _1, server), false);
	server->start();
	scan_sub = n.subscribe("scan", 100, scanCallback);
	robot_pose = n.subscribe("/robot_pose", 1000, poseCallback);
	while (ros::ok()){
		if (server->isPreemptRequested() && state != IDLE) state = PREEMPTED;
		if (state == STOPPING){
			base_cmd.linear.x = base_cmd.angular.z = 0;
			cmd_vel.publish(base_cmd);
			state = IDLE;
		} 
		ros::spinOnce();
		usleep(30000);
	}
}
