#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <strands_navigation_msgs/MonitoredNavigationAction.h>
#include <actionlib/server/simple_action_server.h>

typedef actionlib::SimpleActionServer<strands_navigation_msgs::MonitoredNavigationAction> Server;
Server *server;

ros::Subscriber scan_sub;
float maxDistance = 3.0;		//max range taken into consideration
float edgeThreshold = 0.4;		//min range step to detect an edge 
float minDoorWidth = 0.60;		
float maxDoorWidth = 0.80;
float doorApproachDistance = 0.5;	//the distance from the door  
float auxiliaryDistance = 0.3;		//used to make the door position more precise
float defaultSpeed = 0.1;		//default forward speed of the robot
int passCounterLimit = 10;		//measurements 
int passCounter = 0;		//measurements 
int maxMisdetections = 50;

typedef enum{
	IDLE,
	APPROACH,
	PASS,
	LEAVE,
	STOPPING,
	PREEMPTED,
	SUCCESS,
	FAIL
}EClimbState;

int misdetections = 0;
EClimbState state = IDLE;
ros::Publisher cmd_vel;
geometry_msgs::Twist base_cmd;
     
void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
	if (state == APPROACH || state == PASS || state == LEAVE){
		size_t num_ranges = scan_msg->ranges.size();
		float x[num_ranges];
		float y[num_ranges];
		float d[num_ranges];
		bool m[num_ranges];
		float dx,dy,da;
		float angle;
		printf("%i\n",misdetections++);
		for (int i = 0; i <= num_ranges; i++){
			angle = scan_msg->angle_min+i*scan_msg->angle_increment;
			d[i] = fmin(scan_msg->ranges[i],maxDistance);
			x[i] = d[i]*cos(angle)+0.07;
			y[i] = d[i]*sin(angle);
			m[i] = true;
		}
		base_cmd.linear.x = 0; 
		base_cmd.angular.z = 0; 
		for (int i = 1; i <= num_ranges-1; i++){
			/*edge of a door detected*/
			if ((d[i]-d[i-1])<-edgeThreshold){
				float minDist = 1000;
				int index=0;
				for (int j=2;j<i;j++){
					dx = (x[j]-x[i]);
					dy = (y[j]-y[i]);
					if (sqrt(dx*dx+dy*dy)<minDist){
						minDist = sqrt(dx*dx+dy*dy);
						index = j;
					}
				}
				/*door parameters*/
				int left = i;
				int right = index;	
				float doorAngle,doorCentreX,doorCentreY,auxX,auxY,doorWidth;
				doorAngle = M_PI/2+atan2(y[left]-y[right],x[left]-x[right]);	
				doorCentreX = (x[right]+x[left])/2;	
				doorCentreY = (y[right]+y[left])/2;
				auxiliaryDistance = 0.3;
				auxX = doorCentreX+cos(doorAngle)*auxiliaryDistance;
				auxY = doorCentreY+sin(doorAngle)*auxiliaryDistance;
				doorWidth = sqrt((x[right]-x[left])*(x[right]-x[left])+(y[right]-y[left])*(y[right]-y[left]));
				printf("Initial: %i %i %f %f %f \n",right,left,doorWidth,auxX,auxY);	
				minDist = 100;
				for (int i = 0;i<right;i++){
					dx = (x[i]-auxX);
					dy = (y[i]-auxY);
					if (sqrt(dx*dx+dy*dy)<minDist){
						minDist = sqrt(dx*dx+dy*dy);
						index = i;
					}
				}
				right = index;
				minDist = 100;
				for (int i = left;i<num_ranges;i++){
					dx = (x[i]-auxX);
					dy = (y[i]-auxY);
					if (sqrt(dx*dx+dy*dy)<minDist){
						minDist = sqrt(dx*dx+dy*dy);
						index = i;
					}	
				}
				left = index;
				doorCentreX = (x[right]+x[left])/2;	
				doorCentreY = (y[right]+y[left])/2;
				doorWidth = sqrt((x[right]-x[left])*(x[right]-x[left])+(y[right]-y[left])*(y[right]-y[left]));
				printf("Corrected: %i %i %f \n",right,left,doorWidth);
				doorAngle = M_PI/2+atan2(y[left]-y[right],x[left]-x[right]);	

				if (doorWidth > minDoorWidth && doorWidth < maxDoorWidth)
				{
					printf("Detected door width %f %f %f %f ",doorWidth,doorCentreX,doorCentreY,doorAngle);
					if (state == APPROACH){
						auxX = doorCentreX + cos(doorAngle)*doorApproachDistance;
						auxY = doorCentreY + sin(doorAngle)*doorApproachDistance;
						printf("Moving to %f %f ",auxX,auxY);
						if (auxX < 0.05) passCounter++; else passCounter=0;
						if (passCounter >  passCounterLimit) state = PASS;
						printf("\n");
						base_cmd.linear.x = fmin(defaultSpeed,auxX); 
						base_cmd.angular.z = auxY; 
					}
					if (state == PASS){
						maxDistance = 1.5;
						printf("Moving to %f %f ",doorCentreX,doorCentreY);
						base_cmd.linear.x = fmax(defaultSpeed-2*fabs(doorCentreY),0); 
						base_cmd.angular.z = doorCentreY; 
						if (doorCentreX < 0.1) passCounter++; else passCounter=0;
						if (passCounter > passCounterLimit) state = LEAVE;
					}
					misdetections=0;
				} 
			}
		}	
		if (state == LEAVE){
			base_cmd.linear.x = defaultSpeed*2; 
			base_cmd.angular.z = 0; 
		}
		cmd_vel.publish(base_cmd);
	}	
}

void actionServerCallback(const strands_navigation_msgs::MonitoredNavigationGoalConstPtr& goal, Server* as)
{
	strands_navigation_msgs::MonitoredNavigationResult result;
	misdetections = 0;
	state = APPROACH;
	maxDistance = 3.0;
	while (state == APPROACH || state == PASS || state == LEAVE){
		if (misdetections > maxMisdetections){
			if (state == LEAVE) state = SUCCESS; else state = FAIL;
		}
		usleep(20000);
	}
	if (state == SUCCESS) server->setSucceeded(result);
	if (state == FAIL) server->setAborted(result);
	if (state == PREEMPTED) server->setPreempted(result);
	state = STOPPING;
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "doorPassing");
	ros::NodeHandle n;
	cmd_vel = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	server = new Server(n, "doorPassing", boost::bind(&actionServerCallback, _1, server), false);
	server->start();
	scan_sub = n.subscribe("scan", 100, scanCallback);
	while (ros::ok()){
		if (server->isPreemptRequested() && state != IDLE) state = PREEMPTED;
		if (state == STOPPING)
		{
			base_cmd.linear.x = base_cmd.angular.z = 0;
			cmd_vel.publish(base_cmd);
			state = IDLE;
		} 
		ros::spinOnce();
		usleep(30000);
	}
}

