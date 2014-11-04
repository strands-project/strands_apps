#include <ros/ros.h>
#include <ros/time.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <mongodb_store/SetParam.h>
#include <boost/thread.hpp>

#include <math.h>

int save;

ros::Publisher mileage_pub;
double total_distance;
geometry_msgs::Point last_point;
ros::ServiceClient client;
mongodb_store::SetParam srv;
int save_interval;
bool updated;
boost::mutex mutex;

bool isUpdated() {
    boost::lock_guard<boost::mutex> lock(mutex);
    return updated;
}

void setUpdated(bool up) {
    boost::lock_guard<boost::mutex> lock(mutex);
    updated = up;
}

void updateMileageCallback(const std_msgs::Float32::ConstPtr &mileage)
{
    ROS_INFO("Update mileage");
    if(mileage->data > total_distance){
        total_distance = mileage->data;
    }
    setUpdated(true);
}

void callback(const nav_msgs::Odometry::ConstPtr &odom)
{
    if(isnan(last_point.x) && isnan(last_point.y) && isnan(last_point.z)) {
        last_point = odom->pose.pose.position;
        return;
    }

    double distance = sqrt(pow(odom->pose.pose.position.x-last_point.x,2)+pow(odom->pose.pose.position.y-last_point.y,2));
    total_distance += distance;

    std_msgs::Float32 pub_dist;
    pub_dist.data = (float) total_distance;
    mileage_pub.publish(pub_dist);

    last_point = odom->pose.pose.position;

    if(save % save_interval == 0) {
        ros::param::set("/saved_mileage",total_distance);
        srv.request.param = "saved_mileage";
        if (client.call(srv))
            ROS_DEBUG("Save mileage: success");
        else
            ROS_WARN("Save mileage: failed");
        save = 0;
    }
    save++;
}

void updateMileage(ros::NodeHandle &n, double timeout, std::string mileage_topic) {
    ROS_INFO("Waiting for mileage update from: %s", mileage_topic.c_str());
    ros::Subscriber sub = n.subscribe(mileage_topic, 1, &updateMileageCallback);
    ros::Time end;
    end.fromSec(ros::Time::now().toSec() + timeout);
    while(!isUpdated() && ros::Time::now() < end) {
        ros::spinOnce();
    }
    sub.shutdown();
    ROS_INFO("%s", isUpdated() ? "Updated from topic" : "Timed out");
}

int main(int argc, char **argv)
{
    // Set up ROS.
    ros::init(argc, argv, "odom_mileage");
    ros::NodeHandle n;

    last_point.x = NAN;
    last_point.y = NAN;
    last_point.z = NAN;

    save = 1;
    updated = false;

    // Declare variables that can be modified by launch file or command line.
    std::string mileage_topic;
    std::string update_mileage_topic;
    std::string odom_topic;
    double timeout;

    // Initialize node parameters from launch file or command line.
    // Use a private node handle so that multiple instances of the node can be run simultaneously
    // while using different parameters.
    ros::NodeHandle private_node_handle_("~");
    private_node_handle_.param("mileage_topic", mileage_topic, std::string("/odom_mileage"));
    private_node_handle_.param("initial_mileage_topic", update_mileage_topic, std::string("/mileage"));
    private_node_handle_.param("odom_topic", odom_topic, std::string("/odom"));
    private_node_handle_.param("save_interval", save_interval, 500);
    private_node_handle_.param("init_update_timeout", timeout, 10.0);
    n.param("/saved_mileage", total_distance, 0.0);
    updateMileage(n, timeout, update_mileage_topic);

    client = n.serviceClient<mongodb_store::SetParam>("/config_manager/save_param");

    //Create a subscriber
    ros::Subscriber odom_sub = n.subscribe(odom_topic.c_str(), 50, &callback);
//    ros::Subscriber sub = n.subscribe("/mileage", 1, &updateMileageCallback);

    // Create a publisher
    mileage_pub = n.advertise<std_msgs::Float32>(mileage_topic.c_str(), 10);

    ros::spin();
    return 0;
}
