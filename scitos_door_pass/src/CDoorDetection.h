#include <ros/ros.h>
#include <std_msgs/String.h>
#include <vector>

typedef struct{
	float doorAngle;
	float doorCentreX,doorCentreY;
	float auxX,auxY;
	float doorWidth;
	bool found;
}SDoor;

SDoor detectDoor(std::vector<float> d,float angle_min,float angle_increment,size_t num_ranges,float maxDistance);
