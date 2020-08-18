#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h" //Required Library For "cmd_vel" Topic Publishing

//Common C std Libraries [2] and Signal Handling Library [3]
#include <stdlib.h>
#include <stdio.h>
#include <signal.h>

//Twist Message to Publish
geometry_msgs::Twist msg;

//Publisher
ros::Publisher vel_pub;


void handler(int sig){ //Signal Handler

  // Linear Vector To 0
  msg.linear.x = 0.0;
  msg.linear.y = 0.0;
  msg.linear.z = 0.0;

  // Angular Vector To 0
  msg.angular.x = 0.0;
  msg.angular.y = 0.0;
  msg.angular.z = 0.0;

  //Publish The Stop Values
  vel_pub.publish(msg);

  //Exit
  exit(0);
}

int main(int argc, char **argv){

 //Initialise ROS Node as "cmd_vel_pub"
 ros::init(argc, argv, "cmd_vel_pub");
 ros::NodeHandle n;

 //Publish geometry_msgs/Twist type to "cmd_vel" topic
 vel_pub  = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
 ros::Rate loop_rate(10);

 //Linear Vector Unused
 msg.linear.x = 0.0;
 msg.linear.y = 0.0;
 msg.linear.z = 0.0;

 signal(SIGINT, handler); //Expecting Interrupt Signal (CTRL-C)
 while (ros::ok()){

   //Set Algular Velocity Around Z axis to 2.0; X and Y Unused
   msg.angular.x = 0.0;
   msg.angular.y = 0.0;
   msg.angular.z = 2.0;

   //Post Angular Velocities To Terminal
   ROS_INFO("Publishing: [X: %f , Y: %f , Z: %f]", msg.angular.x, msg.angular.y, msg.angular.z);

   //Publish To Topic
   vel_pub.publish(msg);

   ros::spinOnce();
   loop_rate.sleep();
 }

 return 0;
}
