#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"

void chatterCallback(const sensor_msgs::LaserScan::ConstPtr& msg){

  ROS_INFO("I heard: Minimum Angle [%f]", msg->angle_min);
  ROS_INFO("I heard: Sequence ID: [%i]", msg->header.seq);
  ROS_INFO("I heard: First Element of Ranges: [%f]", msg->ranges[0]);
}

int main(int argc, char **argv){
  ros::init(argc, argv, "laser_sub");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("base_scan", 1, chatterCallback);
  ros::spin();
  return 0;
}
