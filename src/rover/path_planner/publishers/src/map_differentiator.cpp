#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <geometry_msgs/Point.h>
#include <sstream>
#include <string>
#include <typeinfo>
#include <fstream>
#include <iostream>
#include <cmath>
#include <vector>
#include <grid_map_msgs/GridMap.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

using namespace std;

ros::Subscriber sub_;
ros::Publisher pub_;
ros::Publisher pc1_pub_;
int half_side = 10;
int scale = 100;

float currentGoalX = 0;
float currentGoalY = 0;

ros::Subscriber elevation_sub_;
ros::Publisher goal_pub_;
ros::Subscriber goal_sub_;

void gridCallback(const grid_map_msgs::GridMap& grid_map);
void callback(const sensor_msgs::PointCloud2& point_cloud);
void goalCallback(const geometry_msgs::Point& point);

bool got_a_goal = false;

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv) {
	
	ros::init(argc, argv, "map_differentiator");

	ros::NodeHandle n;

	sub_ = n.subscribe("/os1_cloud_node/points", 1, callback);
	pub_ = n.advertise<sensor_msgs::PointCloud2>("/point_cloud_differentiated", 1);
	//pc1_pub_ = n.advertise<sensor_msgs::PointCloud>("/pc1", 10);
	goal_pub_ = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
	goal_sub_ = n.subscribe("/goal_point", 1, goalCallback);
  
	ros::Rate loop_rate(1);
	
	int count = 0;
	while(ros::ok()) {
		cout << "Ros ok" << endl;
		geometry_msgs::PoseStamped pose;
		pose.header.seq = count;
		pose.header.stamp = ros::Time::now();
		pose.header.frame_id = "odom";

		pose.pose.position.x = currentGoalX;
		pose.pose.position.y = currentGoalY;
		pose.pose.position.z = 0.0;

		pose.pose.orientation.x = 0.0;
		pose.pose.orientation.y = 0.0;
		pose.pose.orientation.z = 0.0;
		pose.pose.orientation.w = 1.0;
		
		if(got_a_goal)
			goal_pub_.publish(pose);
		
		ros::spinOnce();

		loop_rate.sleep();
		
		count++;
	}
  
	//ros::spin();
	
	
  
	//TODO write in while below : Order global planner to replan every few seconds
  
	/*ros::Rate loop_rate(10);

	int count = 0;
	while (ros::ok()) {
		//This is a message object. You stuff it with data, and then publish it.
		std_msgs::String msg;

		std::stringstream ss;
		ss << "hello world " << count;
		msg.data = ss.str();

		ROS_INFO("%s", msg.data.c_str());

		pub_.publish(msg);

		ros::spinOnce();

		loop_rate.sleep();
		++count;
	}*/

	return 0;
}

void goalCallback(const geometry_msgs::Point& point) {
	currentGoalX = point.x;
	currentGoalY = point.y;
	got_a_goal = true;
}

void callback(const sensor_msgs::PointCloud2& point_cloud) {
	std::cout << "A grid whispering tales of lost souls..." << std::endl;
	
	sensor_msgs::PointCloud point_cloud_1;
	sensor_msgs::convertPointCloud2ToPointCloud(point_cloud, point_cloud_1);
	
	float minX = 200000000;
	float minY = 200000000;
	float minZ = 200000000;
	float maxX = -200000000;
	float maxY = -200000000;
	float maxZ = -200000000;
	
	int width = 2*half_side*scale;
	int height = width;
	int offset = width/2;
	vector<vector<float>> discrete(height, vector<float>(width, 0));
	
	for(int i = 0 ; i < point_cloud_1.points.size(); ++i){
		geometry_msgs::Point32 point;

		point.x = point_cloud_1.points[i].x;
		point.y = point_cloud_1.points[i].y;
		point.z = point_cloud_1.points[i].z;
		
		int index_x = offset + round(point.x*scale);
		int index_y = offset + round(point.y*scale);
		if(index_x < 0)
			index_x = 0;
		if(index_y < 0)
			index_y = 0;
		if(index_x >= width)
			index_x = width - 1;
		if(index_y >= height)
			index_y = height - 1;
			
		discrete[index_y][index_x] = point.z;
		
		if(point.x < minX)
			minX = point.x;
		if(point.x > maxX)
			maxX = point.x;
		if(point.y < minY)
			minY = point.y;
		if(point.y > maxY)
			maxY = point.y;
		if(point.z < minZ)
			minZ = point.z;
		if(point.z > maxZ)
			maxZ = point.z;
		
		/*std::cout << "x : " << point.x << std::endl;
		std::cout << "y : " << point.y << std::endl;
		std::cout << "z : " << point.z << std::endl << std::endl;*/
		//TODO remove this!
		//point_cloud_1.points[i].z = -0.5f;
	}
	
	float diff = (maxZ - minZ)/7;
	diff = 0.2;
	cout << "-----------DIFF : " << diff << endl;
	int cvg = 7;
	vector<vector<float>> discrete_out(height, vector<float>(width, 0));
	for (unsigned int iy = 12; iy < height - 12; iy++) {
		for (unsigned int ix = 12; ix < width - 12; ix++) {
			if(discrete[iy][ix] != 0) {
				for(unsigned int j = iy - cvg; j < iy + cvg; j++) {
					for(unsigned int i = ix - cvg; i < ix + cvg; i++) {
						float val = discrete[j][i];
						if(val != 0 && abs(discrete[iy][ix] - val) > diff) {
							for(int y = iy - 1; y <= iy + 1; y++) {
								for(int x = ix - 1; x <= ix + 1; x++) {
									discrete_out[y][x] = 1;
								}
							}
						}
					}
				}
			}
		}
	}
	
	for(int i = 0 ; i < point_cloud_1.points.size(); ++i){
		geometry_msgs::Point32 point;
		point.x = point_cloud_1.points[i].x;
		point.y = point_cloud_1.points[i].y;
		
		int index_x = offset + round(point.x*scale);
		int index_y = offset + round(point.y*scale);
		
		if(discrete_out[index_y][index_x] == 1) 
			point_cloud_1.points[i].z = discrete_out[index_y][index_x];
		else
			point_cloud_1.points[i].z = -0.5f;
	}
	
	/*std::ofstream ostream;
	ostream.open("/home/ros-industrial/Desktop/discrete_cloud.pgm");
	ostream << "P2" << std::endl;
	ostream << width << " " << height << std::endl;
	ostream << 255 << std::endl;

	for (unsigned int iy = 0; iy < height; iy++) {
		for (unsigned int ix = 0; ix < width; ix++) {
			if(discrete_out[iy][ix] == 0)
				ostream << 0 << " ";
			else {
				ostream << (int)(discrete_out[iy][ix]*200) << " ";
			}
		}
		ostream << std::endl;
	}
	ostream.close();
	
	std::cout << "minX : " << minX << std::endl;
	std::cout << "minY : " << minY << std::endl;
	std::cout << "maxX : " << maxX << std::endl;
	std::cout << "maxY : " << maxY << std::endl;
	std::cout << "maxZ : " << maxZ << std::endl;
	std::cout << "minZ : " << minZ << std::endl;*/
	
	sensor_msgs::PointCloud2 new_point_cloud;
	sensor_msgs::convertPointCloudToPointCloud2(point_cloud_1, new_point_cloud);
	
	//pc1_pub_.publish(point_cloud_1);
	pub_.publish(new_point_cloud);
	
	/*sensor_msgs::PointCloud2 output;
	
	output.header = point_cloud.header;
	output.height = point_cloud.height;
	output.width = point_cloud.width;
	output.fields = point_cloud.fields;
	output.is_bigendian = point_cloud.is_bigendian;
	output.point_step = point_cloud.point_step;
	output.row_step = point_cloud.row_step;
	
	for(auto f : point_cloud.fields) {
		std::cout << "Field : " << f << " " << f.name << std::endl; 
	}
	
	size_t size = point_cloud.row_step * point_cloud.height;
	std::vector<uint8_t> vect(size);
	for(int i = 0; i < size; i++) {
		vect[i] = 2;
	}
	output.data = vect;
	output.is_dense = point_cloud.is_dense;
	
	std::cout << "width, height : " << point_cloud.width << " " << point_cloud.height << std::endl;
	std::cout << "point_cloud.point_step : " << point_cloud.point_step << std::endl;
	std::cout << "point_cloud.row_step : " << point_cloud.row_step << std::endl;
	std::cout << " vector.size: " << point_cloud.data.size() << std::endl;*/
	
	
	/*std::ofstream output_stream;
	//output_stream.open("point_cloud.pgm");
	output_stream.open("/home/ros-industrial/Desktop/point_cloud.pgm");
	output_stream << "P2" << std::endl;
	output_stream << point_cloud.height << " " << point_cloud.row_step << std::endl;
	output_stream << 255 << std::endl;

	for (unsigned int iy = 0; iy < point_cloud.height; iy++) {
		for (unsigned int ix = 0; ix < point_cloud.row_step; ix++) {
			output_stream << (int)(point_cloud.data[iy*point_cloud.row_step+ix]) << " ";
		}
		output_stream << std::endl;
	}
	output_stream.close();
	
	//pub_.publish(output);
	std::cout << "Published the copied point cloud" << std::endl;*/
	
}
