/*
 * converterPWC.cpp
 *
 *  Created on: Nov 25, 2020
 *      Author: Etienne Salimbeni
 *   Institute: Xplore EPFL
 */

#include <ros/ros.h>

#include "std_msgs/String.h"

#include "nav_msgs/Odometry.h"

#include "geometry_msgs/PoseWithCovarianceStamped.h"



int main(int argc, char** argv) {
  
  
  ros::init(argc, argv, "converter_PWC");

  ros::NodeHandle n;

  ros::Publisher etienne_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("PWC", 1000);


  auto callbackOdom = [&](const nav_msgs::Odometry::ConstPtr& msg){
    geometry_msgs::PoseWithCovarianceStamped output;
    output.pose = msg->pose;
    output.header = msg->header;
    etienne_pub.publish(output); 
    ros::spinOnce();
  };


  // subscriber to /odom

  ros::Subscriber sub = n.subscribe<nav_msgs::Odometry>("/odom", 1000, callbackOdom );
  ros::spin();
  return 0;
}