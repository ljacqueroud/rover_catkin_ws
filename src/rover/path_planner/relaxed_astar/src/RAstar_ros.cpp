/* iPath: A C++ Library of Intelligent Global Path Planners for Mobile Robots with ROS Integration. 

 * Website: http://www.iroboapp.org/index.php?title=IPath
 * Contact: 
 *
 * Copyright (c) 2014
 * Owners: Al-Imam University/King AbdulAziz Center for Science and Technology (KACST)/Prince Sultan University
 * All rights reserved.
 *
 * License Type: GNU GPL
 *
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h> 
#include <fstream>
#include <iostream>
#include <iomanip>
#include <string>

#include <cmath>
#include <geometry_msgs/Twist.h>
#include <grid_map_msgs/GridMap.h>
#include "std_msgs/String.h"
//#include "publishers/AddTwoInts.h"
#include <cstdlib>
#include <random>

#include "RAstar_ros.h"

#include <pluginlib/class_list_macros.h>

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(RAstar_planner::RAstarPlannerROS, nav_core::BaseGlobalPlanner)

//==========================
float* OGM_continuous;
//==========================

int value;
int mapSize;
bool* OGM;
static const float INFINIT_COST = INT_MAX; //!< cost of non connected nodes
float infinity = std::numeric_limits< float >::infinity();
float tBreak;  // coefficient for breaking ties
ofstream MyExcelFile ("RA_result.xlsx", ios::trunc);

int clock_gettime(clockid_t clk_id, struct timespect *tp);

timespec diff(timespec start, timespec end) {
	timespec temp;
	if ((end.tv_nsec-start.tv_nsec)<0) {
		temp.tv_sec = end.tv_sec-start.tv_sec-1;
		temp.tv_nsec = 1000000000+end.tv_nsec-start.tv_nsec;
	} else {
		temp.tv_sec = end.tv_sec-start.tv_sec;
		temp.tv_nsec = end.tv_nsec-start.tv_nsec;
	}
	return temp;
}

inline vector <int> findFreeNeighborCell (int CellID);

namespace RAstar_planner {
	
//Default Constructor
RAstarPlannerROS::RAstarPlannerROS() {
	cout << "========Constructed via default constructor" << endl;
}

RAstarPlannerROS::RAstarPlannerROS(ros::NodeHandle &nh) {
	ROSNodeHandle = nh;
	cout << "============Constructed via nodeHandle constructor" << endl;
}

RAstarPlannerROS::RAstarPlannerROS(std::string name, costmap_2d::Costmap2DROS* global_costmap) {//costmap_ros)
	initialize(name, global_costmap); //costmap_ros);
}

void chatterCallback(const std_msgs::String::ConstPtr& msg) {
	ROS_INFO("I heard: [%s]", msg->data.c_str());
	//TODO : simply do sth like : this.map_ = map_;
}

void twistCallback(const geometry_msgs::Twist& msg) {
	cout << "I heard something" << endl;
}

int polynomial_degree = 2;
vector<float> angles;
vector<float> costs;
void generateCostFunction(vector<float>& x, vector<float>& y, 
		vector<float>& result_poly, int polynomial_degree=2) {
	cout << "GENERATING COST FUNCTION..." << endl;
	int i,j,k;
	int n = polynomial_degree; //Polynomial degree
	int N = x.size(); //Number of data pairs
	if(y.size() != N) {
		cout << "Cannot generate cost function : vectors of different sizes" << endl;
	}
    cout.precision(4);                        //set precision
    cout.setf(ios::fixed);
    
    float X[2*n+1];                        //Array that will store the values of sigma(xi),sigma(xi^2),sigma(xi^3)....sigma(xi^2n)
    for (i=0;i<2*n+1;i++) {
        X[i]=0;
        for (j=0;j<N;j++)
            X[i]=X[i]+pow(x[j],i);        //consecutive positions of the array will store N,sigma(xi),sigma(xi^2),sigma(xi^3)....sigma(xi^2n)
    }
    float B[n+1][n+2],a[n+1];            //B is the Normal matrix(augmented) that will store the equations, 'a' is for value of the final coefficients
    for (i=0;i<=n;i++)
        for (j=0;j<=n;j++)
            B[i][j]=X[i+j];            //Build the Normal matrix by storing the corresponding coefficients at the right positions except the last column of the matrix
    float Y[n+1];                    //Array to store the values of sigma(yi),sigma(xi*yi),sigma(xi^2*yi)...sigma(xi^n*yi)
    for (i=0;i<n+1;i++) {    
        Y[i]=0;
        for (j=0;j<N;j++)
        Y[i]=Y[i]+pow(x[j],i)*y[j];        //consecutive positions will store sigma(yi),sigma(xi*yi),sigma(xi^2*yi)...sigma(xi^n*yi)
    }
    for (i=0;i<=n;i++)
        B[i][n+1]=Y[i];                //load the values of Y as the last column of B(Normal Matrix but augmented)
    n=n+1;                //n is made n+1 because the Gaussian Elimination part below was for n equations, but here n is the degree of polynomial and for n degree we get n+1 equations
    
    cout<<"\nThe Normal(Augmented Matrix) is as follows:\n";    
    for (i=0;i<n;i++) {           //print the Normal-augmented matrix
        for (j=0;j<=n;j++)
            cout<<B[i][j]<<setw(16);
        cout<<"\n";
    }    
    for (i=0;i<n;i++)                    //From now Gaussian Elimination starts(can be ignored) to solve the set of linear equations (Pivotisation)
        for (k=i+1;k<n;k++)
            if (B[i][i]<B[k][i])
                for (j=0;j<=n;j++) {
                    float temp=B[i][j];
                    B[i][j]=B[k][j];
                    B[k][j]=temp;
                }
     
    for (i=0;i<n-1;i++)            //loop to perform the gauss elimination
        for (k=i+1;k<n;k++) {
                float t=B[k][i]/B[i][i];
                for (j=0;j<=n;j++)
                    B[k][j]=B[k][j]-t*B[i][j];    //make the elements below the pivot elements equal to zero or elimnate the variables
        }
            
    for (i=n-1;i>=0;i--) {                //back-substitution
							//x is an array whose values correspond to the values of x,y,z..
        a[i]=B[i][n];                //make the variable to be calculated equal to the rhs of the last equation
        for (j=0;j<n;j++)
            if (j!=i)            //then subtract all the lhs values except the coefficient of the variable whose value                                   is being calculated
                a[i]=a[i]-B[i][j]*a[j];
        a[i]=a[i]/B[i][i];            //now finally divide the rhs by the coefficient of the variable to be calculated
    }
    
    cout<<"\nThe values of the coefficients are as follows:\n";
    for (i=0;i<n;i++)
        cout<<"x^"<<i<<"="<<a[i]<<endl;            // Print the values of x^0,x^1,x^2,x^3,....    
    cout<<"\nHence the fitted Polynomial is given by:\ny=";
    for (i=0;i<n;i++) {
        cout<<" + ("<<a[i]<<")"<<"x^"<<i;
        result_poly.push_back(a[i]);
	}
    cout<<"\n";
}

bool cumulate_energy = false; //might be useless
float energy_ = 0;
void energyCallback(const std_msgs::String::ConstPtr& msg) {
	ROS_INFO("ENERGY CALLBACK");
	if(energy_ < 0) {
		ROS_WARN("Energy call back : energy should not be negative!");
	}
	//TODO get energy from the message
	float energy = 0;
	//energy_ = cumulate_energy ? (energy_ + energy) : energy;
	energy_ += energy;
}

struct Pose {      
   float x, y, z;      
};

float get_distance(const Pose& pose1, const Pose& pose2) {
	float dx = pose1.x - pose2.x;
	float dy = pose1.y - pose2.y;
	return sqrt(dx*dx + dy*dy);
}

float get_angle(const float distance, const int cellId1, const int cellId2) {
	//TODO compute respective heights of pose1 and 2, deduce angle
	float h1 = OGM_continuous[cellId1];
	float h2 = OGM_continuous[cellId2];
	float height_diff = h2 - h1;
	return atan(height_diff / distance);
}

//distribution params
const double mean = 0.0;
const double stddev = 10;
std::default_random_engine generator;
std::normal_distribution<double> dist(mean, stddev);
void generate_test_cost_and_angle(float& cost, float& angle) {
	//generate numbers in [-10°, 20°]
	angle = static_cast<float> (rand()) / static_cast<float>(RAND_MAX) * M_PI / 6 - (M_PI/18);
	//cost with noise
	cost = 391 * max(0.0f, angle) + 175 + dist(generator); //6.83
}

Pose prev_pose;
int prev_pose_cell = -1;
#define generate_cost_function_wait_cycles 100
int generate_cost_function_counter = 0;
vector<float> cost_function_poly;
bool computed_cost_function = false;
void RAstarPlannerROS::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
	cout << endl << endl;
	ROS_INFO("ODOM CALLBACK");
	
	//==== Maybe this is useless ====
	tf::Pose pose_tf;
	poseMsgToTF(msg->pose.pose, pose_tf);
	//===============================
	
	float x = msg->pose.pose.position.x;
	float y = msg->pose.pose.position.y;
	float z = msg->pose.pose.position.z; //might be useful to compute height differences
	
	ROS_INFO("Seq: [%d]", msg->header.seq);
	ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", x, y, z);
	ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
	ROS_INFO("Vel-> Linear: [%f], Angular: [%f]", msg->twist.twist.linear.x,msg->twist.twist.angular.z);

	// convert the start and goal positions
	float posX = x;
	float posY = y;
	Pose current_pose;
	int current_pose_cell;
	getCoordinate(posX, posY);

	if (RAstarPlannerROS::isCellInsideMap(posX, posY)) {
		ROS_INFO("cell is inside map");
		current_pose_cell = convertToCellIndex(posX, posY);
		current_pose = {posX, posY, 0};
		if(prev_pose_cell == -1) {
			ROS_INFO("prev_pose_cell = -1");
			prev_pose_cell = current_pose_cell;
			prev_pose = current_pose;
			return;
		}
	} else {
		ROS_ERROR("the position given by odometry appears to be outside the map");
		return; //TODO : do something else maybe?
	}
	
	//int x_cell = getCellRowID(current_pose_cell);
	//int y_cell = getCellColID(current_pose_cell);
	
	if(current_pose_cell == prev_pose_cell) {
		//cumulate_energy = true; //might be useless
		ROS_INFO("Have not changed cells");
		//TODO un comment the return
		//return;
	}
	ROS_WARN("COMPUTING POINT FOR POLYNOMIAL FUNCTION");
	float distance = get_distance(current_pose, prev_pose);
	float cost = energy_/distance;
	float angle = get_angle(distance, prev_pose_cell, current_pose_cell);
	
	//TODO remove this, it's just for testing
	generate_test_cost_and_angle(cost, angle);
	
	angles.push_back(angle);
	costs.push_back(cost);
	
	prev_pose = current_pose;
	prev_pose_cell = current_pose_cell;
	generate_cost_function_counter++;
	//cumulate_energy = false;
	energy_ = 0;
	
	//TODO uncomment this if and remove the next one
	/*if(generate_cost_function_counter > generate_cost_function_wait_cycles) {
		cost_function_poly.clear();
		generateCostFunction(slopes, costs, cost_function_poly);
		cout << "Cost function polynomial : " << cost_function_poly.size() << endl;
		for (int i=0;i<cost_function_poly.size();i++) {
			cout<<" + ("<<cost_function_poly[i]<<")"<<"x^"<<i;
		}
		cout << endl << flush;
		generate_cost_function_counter = 0;
	}*/
	if(costs.size() == 100 && !computed_cost_function) {
		computed_cost_function = true;
		cost_function_poly.clear();
		generateCostFunction(angles, costs, cost_function_poly);
		cout << "Cost function polynomial : " << cost_function_poly.size() << endl;
		for (int i=0;i<cost_function_poly.size();i++) {
			cout<<" + ("<<cost_function_poly[i]<<")"<<"x^"<<i;
		}
		cout << endl << flush;
		
		ofstream output_stream;
		output_stream.open("polynomial_function.txt");
		for(int i=0;i<cost_function_poly.size();i++) {
			output_stream<<" + ("<<cost_function_poly[i]<<")"<<"x^"<<i << flush;
		}
		output_stream << endl << endl;
		for(int i = 0; i < costs.size(); i++) {
			output_stream << "{"<<angles[i]<<", "<<costs[i]<<"}, ";
		}
		output_stream << endl << endl;
		for(int i = 0; i < costs.size(); i++) {
			output_stream <<angles[i]<<", " << flush;
		}
		output_stream << endl;
		for(int i = 0; i < costs.size(); i++) {
			output_stream <<costs[i]<<", " << flush;
		}
		output_stream << endl;
		output_stream.close();
	}
}

//std_msgs::Float32MultiArray[] data;
int h_;
int w_;
float nan_value = 0.0000123;
void gridCallback(const grid_map_msgs::GridMap& grid_map) {
	cout << "I heard a grid whisper to me tales of lost souls..." << endl;
	//std_msgs::Float32MultiArray msg = grid_map.data;
	auto grid_msg = grid_map.data;
	auto msg = grid_msg[0];
	
	//cout << "layout : " << msg.layout << endl;
	
	int dstride0 = msg.layout.dim[0].stride;
	int dstride1 = msg.layout.dim[1].stride;
	h_ = msg.layout.dim[0].size;
	w_ = msg.layout.dim[1].size;
	
	cout << "dstride0 : " << dstride0 << endl;
	cout << "dstride1 : " << dstride1 << endl;
	cout << "offset 0 : " << grid_map.outer_start_index << endl;
	cout << "offset 1 : " << grid_map.inner_start_index << endl;
	
	/*ROS_INFO("mat(0,0) = %f",msg.data[0 + dstride1*0]);
	ROS_INFO("mat(0,1) = %f",msg.data[0 + dstride1*1]);
	ROS_INFO("mat(1,1) = %f\r\n",msg.data[1 + dstride1*1]);*/
	/*for(int y(0); y < h ; y++) {
		for(int x(0); x < w; x++) {
			float val = msg.data[x + dstride1*y];
			val = isnan(val) ? 0 : val;
			ROS_INFO("mat(%d,%d) = %f", x, y, val);
		}
	}*/
	
	int size = w_ * h_;
	OGM_continuous = new float[size];
	float max_reached = 0;
	
	int gry = (grid_map.inner_start_index >= dstride1/2)? grid_map.inner_start_index - dstride1 : grid_map.inner_start_index;
	int grx = (grid_map.outer_start_index >= dstride1/2)? grid_map.outer_start_index - dstride1 : grid_map.outer_start_index;
	cout << "grx : " << grx << endl;
	cout << "gry : " << gry << endl;
	
	//for (unsigned int iy = h_-1; iy < h_; iy--) { //Tricky condition b/c unsigned int is never negative
	for(unsigned int iy = 0; iy < h_; iy++) {
		for (unsigned int ix = 0; ix < w_; ix++) {
			float val = msg.data[ix + dstride1*iy];
			if(iy == 0 && ix == 0 )
				cout << msg.data[0] << " .........................." << endl;
			val = isnan(val) ? nan_value : val; ///100.0f; // /100.0 to convert [cm] to [m]
			//val *= 100;
			//ROS_INFO("mat(%d,%d) = %f", ix, iy, val);
			//OGM_continuous[iy*w_+ix] = max(0, (int) val);
			float final_val = max(0.0f, val);
			if(final_val > max_reached) max_reached = final_val;
			//OGM_continuous[(h_ - 1 - iy)+ix*w_] = final_val;
			//OGM_continuous[iy+(w_ - 1 - ix)*w_] = final_val; //need rotation (map should rotate 90° anticlock to match)
			//OGM_continuous[ix+iy*w_] = final_val; //need to rotatio 180°
			int x_ = (w_ - 1 - ix) + grx; //-5*4/*+ grid_map.inner_start_index + dstride1/2*/; //)%dstride1;
			int y_ = (h_ - 1 - iy) + gry; //10*4/*+ grid_map.outer_start_index /*+ dstride1/2*/;
			while(x_ < 0) x_ += 120;
			while(x_ >= 120) x_ -= 120;
			while(y_ < 0) y_ += 120;
			while(y_ >= 120) y_ -= 120;
			bool invalidate = false;
			for(int ey = iy - 1; ey <= iy + 1; ey++) {
				for(int ex = ix - 1; ex <= ix + 1; ex++) {
					if(ey >= 0 && ex >= 0 && ey < h_ && ex < w_) {
						float other_val = msg.data[ex + dstride1*ey];
						if(abs(other_val - val) > 0.2 && !isnan(other_val)) { //TODO need to probably tweak this value
							invalidate = true;
						}
					}
				}
			}
			if(invalidate && final_val != nan_value) {
				for(int ey = y_ - 1; ey <= y_ + 1; ey++) {
					for(int ex = x_ - 1; ex <= x_ + 1; ex++) {
						if(ey >= 0 && ex >= 0 && ey < h_ && ex < w_) {
							OGM_continuous[ex+ey*w_] = 20; //TODO maybe increase this value
						}
					}
				}
			} else {
				OGM_continuous[x_+y_*w_] = final_val;
			}
			
			//might want to consider the outer and inner start index
		}
	}
	ROS_INFO("MAX REACHED : %f", max_reached); 
	/*ofstream output_stream;
	output_stream.open("/home/ros-industrial/Desktop/callback_output.pgm");
	output_stream << "P2" << endl;
	output_stream << h_ << " " << w_ << endl;
	output_stream << (int)(max_reached * 100) << endl;

	for (unsigned int iy = 0; iy < h_; iy++) {
		for (unsigned int ix = 0; ix < w_; ix++) {
			output_stream << (int)(OGM_continuous[iy*w_+ix] * 99) << " ";
		}
		output_stream << endl;
	}
	output_stream.close();
	ROS_INFO("SHOULD HAVE WRITTEN THE callback_output.pgm"); */
}

ros::Subscriber sub_;
ros::Publisher plan_pub_;
ros::Subscriber energy_sub_;
ros::Subscriber odom_sub_;

float offX; // = -9.75; //TODO this is to mitigate the shift when loading at different coordinates than (0, 0)
float offY;
float decalageX;
float decalageY;
void RAstarPlannerROS::initialize(std::string name, costmap_2d::Costmap2DROS* global_costmap) {//costmap_ros)

	cout << "==============Got asked to initialize the planner with name : " << name << endl;

	if (!initialized_) {
		costmap_ros_ = global_costmap; //costmap_ros;
		costmap_ = costmap_ros_->getCostmap();
		width = costmap_->getSizeInCellsX();
		height = costmap_->getSizeInCellsY();
		resolution = costmap_->getResolution();
		
		offX = -(width*resolution/2);
		offY = -(height*resolution/2);
		cout << "offX prev : " << offX << endl;
		cout << "offY prev : " << offY << endl;

		ros::NodeHandle private_nh("~/" + name);
		//Maybe this will publish the plan somewhere somehow
		plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);

		originX = costmap_->getOriginX();
		originY = costmap_->getOriginY();
		decalageX = originX - offX;
		decalageY = originY - offY;
		cout << "origin X : " << originX << endl;
		cout << "origin Y : " << originY << endl;
		cout << "decalageX : " << decalageX << endl;
		cout << "decalageY : " << decalageY << endl;
		
		originX = offX;
		originY = offY;

		//////
		//sub_ = private_nh.subscribe("/cmd_vel", 1000, twistCallback);
		//sub_ = private_nh.subscribe("/elevation_mapping/elevation_map", 1000, gridCallback);
		sub_ = private_nh.subscribe("/elevation_mapping/elevation_map_raw", 1, gridCallback);
		
		energy_sub_ = private_nh.subscribe("/motors/consumed_energy", 1, energyCallback);
		//odom_sub_ = private_nh.subscribe("/odom", 1000, &RAstarPlannerROS::odomCallback, this);
		
		//ros::spinOnce();
		
		cout << "------------- Map    Topic : " << sub_.getTopic() << "; OUT OF SPIN ----------------------------" << endl;
		cout << "------------- Energy Topic : " << energy_sub_.getTopic() << "; OUT OF SPIN ----------------------------" << endl;
		
		//////

		cout << "width, height, res : " << width << " " << height << " " << resolution << endl;
		mapSize = width*height;
		tBreak = 1+1/(mapSize); 
		value =0;

		ofstream myfile;
		myfile.open("my_output_map.pgm"); //outputs to /home/ros-industrial/.ros/my_output_map.txt
		myfile << "P2" << endl;
		myfile << costmap_->getSizeInCellsX() << " " << costmap_->getSizeInCellsY() << endl;
		myfile << 255 << endl;


		for (unsigned int iy = 0; iy < costmap_->getSizeInCellsY(); iy++) {
			//for (unsigned int iy = costmap_->getSizeInCellsY() - 1; iy >= 0; iy--)
			for (unsigned int ix = 0; ix < costmap_->getSizeInCellsX(); ix++) {
				unsigned int cost = static_cast<int>(costmap_->getCost(ix, iy));
				myfile << cost << " ";
			}
			myfile << endl;
		}

		OGM = new bool [mapSize]; 
		for (unsigned int iy = 0; iy < costmap_->getSizeInCellsY(); iy++) {
			for (unsigned int ix = 0; ix < costmap_->getSizeInCellsX(); ix++) {
				unsigned int cost = static_cast<int>(costmap_->getCost(ix, iy));
				if (cost == 0)
					OGM[iy*width+ix]=true;
				else
					OGM[iy*width+ix]=false;
			}
		}

		myfile.close();

		//===============================

		int rows, cols, size, gray;
		string format;

		ifstream input;
		input.open("input_p2.pgm"); //input from /home/ros-industrial/.ros/input_p2.pgm
		input >> format >> rows >> cols >> gray;
		size = rows * cols;

		ofstream output_stream;
		output_stream.open("output_p2.pgm");
		output_stream << "P2" << endl;
		output_stream << rows << " " << cols << endl;
		output_stream << 255 << endl;

		/*OGM_continuous = new int[size];
		for (unsigned int iy = cols-1; iy < cols; iy--) { //Tricky condition b/c unsigned int is never negative
		//for (unsigned int iy = 0; iy < cols; iy++)
			for (unsigned int ix = 0; ix < rows; ix++) {
				input >> OGM_continuous[iy*width+ix];
			}
		}
		for (unsigned int iy = 0; iy < cols; iy++) {
			for (unsigned int ix = 0; ix < rows; ix++) {
				output_stream << OGM_continuous[iy*width+ix] << " ";
			}
			output_stream << endl;
		}*/

		input.close();
		output_stream.close();


		MyExcelFile << "StartID\tStartX\tStartY\tGoalID\tGoalX\tGoalY\tPlannertime(ms)\tpathLength\tnumberOfCells\t" << endl;

		ROS_INFO("RAstar planner initialized successfully");
		initialized_ = true;
	} else {
		ROS_WARN("This planner has already been initialized... doing nothing");
	}
}

std::string frame_id_;

bool RAstarPlannerROS::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
							 std::vector<geometry_msgs::PoseStamped>& plan) {

	cout << "===============Got asked to makePlan(); " << endl;

	if (!initialized_) {
		ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
		return false;
	}

	ROS_INFO("Got a start: %.2f, %.2f, and a goal: %.2f, %.2f", start.pose.position.x, start.pose.position.y,
			goal.pose.position.x, goal.pose.position.y);

	/////
	/*for (unsigned int iy = 0; iy < costmap_->getSizeInCellsY()/2; iy++) {
		for (unsigned int ix = 0; ix < costmap_->getSizeInCellsX()/2; ix++) {
			costmap_->setCost(ix, iy, 254);
		}
	}*/
	//costmap_->saveMap("map_generated_with_save_map");
	/////

	plan.clear();

	frame_id_ = goal.header.frame_id;
	if (goal.header.frame_id != costmap_ros_->getGlobalFrameID()) {
		ROS_ERROR("This planner as configured will only accept goals in the %s frame, but a goal was sent in the %s frame.",
			  costmap_ros_->getGlobalFrameID().c_str(), goal.header.frame_id.c_str());
		return false;
	}

	tf::Stamped < tf::Pose > goal_tf;
	tf::Stamped < tf::Pose > start_tf;

	poseStampedMsgToTF(goal, goal_tf);
	poseStampedMsgToTF(start, start_tf);

	// convert the start and goal positions

	float startX = start.pose.position.x;
	float startY = start.pose.position.y;

	float goalX = goal.pose.position.x;
	float goalY = goal.pose.position.y;

	getCoordinate(startX, startY);
	getCoordinate(goalX, goalY);

	int startCell;
	int goalCell;
	
	cout << "startX : " << startX << ", startY : " << startY << endl;
	cout << "goalX : " << goalX << ", goalY : " << goalY << endl;
	
	startX = startX - decalageX;
	startY = startY - decalageY;
	
	//TODO 1.3.6 : put this back if the map generated by lidar follows the robot's frame
	startX = startX - start.pose.position.x;
	startY = startY - start.pose.position.y;
	
	goalX = goalX - decalageX;
	goalY = goalY - decalageY;
	
	//TODO 1.3.6 : put this back if the map generated by lidar follows the robot's frame
	goalX = goalX - start.pose.position.x;
	goalY = goalY - start.pose.position.y;

	cout << "startX : " << startX << ", startY : " << startY << endl;
	cout << "goalX : " << goalX << ", goalY : " << goalY << endl;
	cout << "width : " << width << ", height : " << height << endl;
	cout << "resolution : " << resolution << endl;
	
	//If goal outside map, find midpoint that is inside the map :
	if(!isCellInsideMap(goalX, goalY)) {
		float deltaX = startX - goalX;
		float deltaY = startY - goalY;
		float distance = sqrt(deltaX * deltaX + deltaY * deltaY);
		float dx = deltaX / distance;
		float dy = deltaY / distance;
		while(!isCellInsideMap(goalX, goalY)) {
			goalX += dx;
			goalY += dy;
			cout << "new goal : (x, y) = (" << goalX << ", " << goalY << ")" << endl;
		}
	}
	
	if (isCellInsideMap(startX, startY) && isCellInsideMap(goalX, goalY)) {
		startCell = convertToCellIndex(startX, startY);
		goalCell = convertToCellIndex(goalX, goalY);

		MyExcelFile << startCell <<"\t"<< start.pose.position.x <<"\t"<< start.pose.position.y
			<<"\t"<< goalCell <<"\t"<< goal.pose.position.x <<"\t"<< goal.pose.position.y;
	} else {
		ROS_WARN("the start or goal is out of the map");
		return false;
	}
	
	//might be useless
	startX = startX + decalageX;
	startY = startY + decalageY;
	goalX = goalX + decalageX;
	goalY = goalY + decalageY;

	/////////////////////////////////////////////////////////

	// call global planner

	if (isStartAndGoalCellsValid(startCell, goalCell)) {
		vector<int> bestPath;
		bestPath.clear();

		bestPath = RAstarPlanner(startCell, goalCell);

		//if the global planner find a path
		
		if (bestPath.size() > 0) {
			
			float pathCost = 0;

			// convert the path
			cout << "=================Converting path of length : " << bestPath.size() << endl;
			int lastIndex = -1;
			int lastXId;
			int lastYId;
			for (int i = 0; i < bestPath.size(); i++) {

				float x = 0.0;
				float y = 0.0;
				float moveCost = 1.41;
				int index = bestPath[i];
				int xId = getCellColID(index);
				int yId = getCellRowID(index);
				
				if(lastIndex == -1){
					lastIndex = index;
					lastXId = xId;
					lastYId = yId;
				} else {
					if(lastXId == xId || lastYId == yId) {
						moveCost = 1;
					}
					
					float h1 = OGM_continuous[lastIndex];
					float h2 = OGM_continuous[index];

					float pen = computePenalty(h1, h2, moveCost, false);
					//cout<<pen<<endl;
					pathCost += moveCost * pen;
					lastIndex = index;
				}

				convertToCoordinate(index, x, y);
				
				//Reconvert to the outside world coordinates
				x += decalageX;
				y += decalageY;
				
				//TODO 1.3.6 : put this back if the map generated by lidar follows the robot's frame
				x += start.pose.position.x;
				y += start.pose.position.y;

				geometry_msgs::PoseStamped pose = goal;

				pose.pose.position.x = x;
				pose.pose.position.y = y;
				pose.pose.position.z = 0.0;

				pose.pose.orientation.x = 0.0;
				pose.pose.orientation.y = 0.0;
				pose.pose.orientation.z = 0.0;
				pose.pose.orientation.w = 1.0;

				plan.push_back(pose);
			}
			
			cout << "The global path cost : "<< pathCost << endl;

			float path_length = 0.0;
			std::vector<geometry_msgs::PoseStamped>::iterator it = plan.begin();
			
			geometry_msgs::PoseStamped last_pose;
			last_pose = *it;
			it++;
			for (; it!=plan.end(); ++it) {
				path_length += hypot(  (*it).pose.position.x - last_pose.pose.position.x, 
						 (*it).pose.position.y - last_pose.pose.position.y );
				last_pose = *it;
			}
			cout <<"The global path length : "<< path_length<< " meters"<<endl;
			MyExcelFile << "\t" <<path_length <<"\t"<< plan.size() <<endl;
			
			//publish the plan
			publishPlan(plan);
			return true;
		} else {
			ROS_WARN("The planner failed to find a path, choose other goal position");
			return false;
		}
	} else {
		ROS_WARN("Not valid start or goal");
		return false;
	}
}

void RAstarPlannerROS::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path) {
	if (!initialized_) {
		ROS_ERROR(
				"This planner has not been initialized yet, but it is being used, please call initialize() before use");
		return;
	}

	//create a message for the plan
	nav_msgs::Path gui_path;
	gui_path.poses.resize(path.size());

	gui_path.header.frame_id = frame_id_;
	gui_path.header.stamp = ros::Time::now();

	// Extract the plan in world co-ordinates, we assume the path is all in the same frame
	for (unsigned int i = 0; i < path.size(); i++) {
		gui_path.poses[i] = path[i];
	}

	plan_pub_.publish(gui_path);
}

void RAstarPlannerROS::getCoordinate(float& x, float& y) {
	x = x - originX;
	y = y - originY;
}

int RAstarPlannerROS::convertToCellIndex(float x, float y) {
	int cellIndex;
	float newX = x / resolution;
	float newY = y / resolution;
	cellIndex = getCellIndex(newY, newX);
	return cellIndex;
}

void RAstarPlannerROS::convertToCoordinate(int index, float& x, float& y) {
	x = getCellColID(index) * resolution;
	y = getCellRowID(index) * resolution;
	x = x + originX;
	y = y + originY;
}

bool RAstarPlannerROS::isCellInsideMap(float x, float y) {
	bool valid = true;
	if(x > (width * resolution) || y > (height * resolution))
		valid = false;
	if(x < 0 || y < 0)
		valid = false;
	return valid;
}

void RAstarPlannerROS::mapToWorld(double mx, double my, double& wx, double& wy) {
	costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();
	wx = costmap->getOriginX() + mx * resolution;
	wy = costmap->getOriginY() + my * resolution;
}

vector<int> RAstarPlannerROS::RAstarPlanner(int startCell, int goalCell) {
   vector<int> bestPath;

	float g_score [mapSize];

	for (uint i=0; i<mapSize; i++)
		g_score[i]=infinity;

	cout << "Before timespec" << endl;
	timespec time1, time2;
	/*take current time here */
	clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time1);
	bestPath = findPath(startCell, goalCell,  g_score);
	clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time2);

	cout<<"Time to generate best global path by Relaxed A* = " << (diff(time1,time2).tv_sec)*1e3 + (diff(time1,time2).tv_nsec)*1e-6 << " microseconds" << endl;
	MyExcelFile <<"\t"<< (diff(time1,time2).tv_sec)*1e3 + (diff(time1,time2).tv_nsec)*1e-6 ;

	return bestPath;
}

/*******************************************************************************/
//Function Name: findPath
//Inputs: the map layout, the start and the goal Cells and a boolean to indicate if we will use break ties or not
//Output: the best path
//Description: it is used to generate the robot free path
/*********************************************************************************/
vector<int> RAstarPlannerROS::findPath(int startCell, int goalCell, float g_score[])
{
	value++;
	vector<int> bestPath;
	vector<int> emptyPath;
	cells CP;

	multiset<cells> OPL;
	int currentCell;

	//calculate g_score and f_score of the start position
	g_score[startCell]=0;
	CP.currentCell=startCell;
	CP.fCost=g_score[startCell]+calculateHCost(startCell,goalCell);

	//add the start cell to the open list
	OPL.insert(CP);
	currentCell=startCell;

	//while the open list is not empty continuie the search or g_score(goalCell) is equal to infinity
	while (!OPL.empty()&& g_score[goalCell]==infinity) {
		//choose the cell that has the lowest cost fCost in the open set which is the begin of the multiset
		currentCell = OPL.begin()->currentCell;
		//remove the currentCell from the openList
		OPL.erase(OPL.begin());
		//search the neighbors of the current Cell
		vector <int> neighborCells; 
		neighborCells=findFreeNeighborCell(currentCell);
		for(uint i=0; i<neighborCells.size(); i++) //for each neighbor v of current cell
		{
			// if the g_score of the neighbor is equal to INF: unvisited cell
			if(g_score[neighborCells[i]]==infinity)
			{
				g_score[neighborCells[i]]=g_score[currentCell]+getMoveCost(currentCell,neighborCells[i]);
				addNeighborCellToOpenList(OPL, neighborCells[i], goalCell, g_score);
			}//end if
		}//end for
	}//end while

	if(g_score[goalCell]!=infinity)  // if g_score(goalcell)==INF : construct path 
	{
		bestPath=constructPath(startCell, goalCell, g_score);
		return bestPath; 
	} else {
		cout << "Failure to find a path !" << endl;
		return emptyPath;
	}
}

/*******************************************************************************/
//Function Name: constructPath
//Inputs: the start and the goal Cells
//Output: the best path
//Description: it is used to construct the robot path
/*********************************************************************************/
vector<int> RAstarPlannerROS::constructPath(int startCell, int goalCell,float g_score[])
{
	vector<int> bestPath;
	vector<int> path;

	path.insert(path.begin()+bestPath.size(), goalCell);
	int currentCell=goalCell;

	while(currentCell!=startCell)
	{ 
		vector <int> neighborCells;
		neighborCells=findFreeNeighborCell(currentCell);

		vector <float> gScoresNeighbors;
		for(uint i=0; i<neighborCells.size(); i++)
			gScoresNeighbors.push_back(g_score[neighborCells[i]]);
		
		int posMinGScore=distance(gScoresNeighbors.begin(), min_element(gScoresNeighbors.begin(), gScoresNeighbors.end()));
		currentCell=neighborCells[posMinGScore];

		//insert the neighbor in the path
		path.insert(path.begin()+path.size(), currentCell);
	}
	for(uint i=0; i<path.size(); i++)
		bestPath.insert(bestPath.begin()+bestPath.size(), path[path.size()-(i+1)]);

	return bestPath;
}

/*******************************************************************************/
//Function Name: calculateHCost
//Inputs:the cellID and the goalCell
//Output: the distance between the current cell and the goal cell
//Description: it is used to calculate the hCost 
/*********************************************************************************/
/*
float RAstarPlannerROS::calculateHCost(int cellID, int goalCell)
{    
  int x1=getCellRowID(goalCell);
  int y1=getCellColID(goalCell);
  int x2=getCellRowID(cellID);
  int y2=getCellColID(cellID);
  
  //if(getNeighborNumber()==4) 
	//The diagonal shortcut distance between two grid points (x1,y1) and (x2,y2) is:
	//  return min(abs(x1-x2),abs(y1-y2))*sqrt(2) + max(abs(x1-x2),abs(y1-y2))-min(abs(x1-x2),abs(y1-y2));
  
  //else
	//manhatten distance for 8 neighbor
	return abs(x1-x2)+abs(y1-y2);
}
*/

/*******************************************************************************/
//Function Name: addNeighborCellToOpenList
//Inputs: the open list, the neighbors Cell, the g_score matrix, the goal cell 
//Output: 
//Description: it is used to add a neighbor Cell to the open list
/*********************************************************************************/
void RAstarPlannerROS::addNeighborCellToOpenList(multiset<cells> & OPL, int neighborCell, int goalCell, float g_score[])
{
	cells CP;
	CP.currentCell=neighborCell; //insert the neighbor cell
	CP.fCost=g_score[neighborCell]+calculateHCost(neighborCell,goalCell);
	OPL.insert(CP);
	//multiset<cells>::iterator it = OPL.lower_bound(CP);
	//multiset<cells>::iterator it = OPL.upper_bound(CP);
	//OPL.insert( it, CP  );
}

  /*******************************************************************************
 * Function Name: findFreeNeighborCell
 * Inputs: the row and columun of the current Cell
 * Output: a vector of free neighbor cells of the current cell
 * Description:it is used to find the free neighbors Cells of a the current Cell in the grid
 * Check Status: Checked by Anis, Imen and Sahar
*********************************************************************************/

vector <int> RAstarPlannerROS::findFreeNeighborCell (int CellID) {
	int rowID=getCellRowID(CellID);
	int colID=getCellColID(CellID);
	int neighborIndex;
	vector <int>  freeNeighborCells;

	for (int i=-1;i<=1;i++) {
		for (int j=-1; j<=1;j++){
		//check whether the index is valid
			if ((rowID+i>=0)&&(rowID+i<height)&&(colID+j>=0)&&(colID+j<width)&& (!(i==0 && j==0))){
				neighborIndex = getCellIndex(rowID+i,colID+j);
				if(isFree(neighborIndex))
					freeNeighborCells.push_back(neighborIndex);
			}
		}
	}
	return freeNeighborCells;
}

/*******************************************************************************/
//Function Name: isStartAndGoalCellsValid
//Inputs: the start and Goal cells
//Output: true if the start and the goal cells are valid
//Description: check if the start and goal cells are valid
/*********************************************************************************/
bool RAstarPlannerROS::isStartAndGoalCellsValid(int startCell,int goalCell)
{ 
	bool isvalid=true;
	bool isFreeStartCell=isFree(startCell);
	bool isFreeGoalCell=isFree(goalCell);
	if (startCell==goalCell) {
		//cout << "The Start and the Goal cells are the same..." << endl; 
		isvalid = false;
	} else {
		if (!isFreeStartCell && !isFreeGoalCell) {
			//cout << "The start and the goal cells are obstacle positions..." << endl;
			isvalid = false;
		} else {
			if (!isFreeStartCell) {
				//cout << "The start is an obstacle..." << endl;
				isvalid = false;
			} else {
				if(!isFreeGoalCell) {
					//cout << "The goal cell is an obstacle..." << endl;
					isvalid = false;
				} else {
					if (findFreeNeighborCell(goalCell).size()==0) {
						//cout << "The goal cell is encountred by obstacles... "<< endl;
						isvalid = false;
					} else {
						if(findFreeNeighborCell(startCell).size()==0) {
							//cout << "The start cell is encountred by obstacles... "<< endl;
							isvalid = false;
						}
					}
				}
			}
		}
	}
	return isvalid;
}

bool use_inferred_cost_function = false;
float cost_function(float angle, bool b) {
	int cost;
	if(use_inferred_cost_function) {
		if(cost_function_poly.size() < 1) {
			ROS_WARN("Got an empty cost_function_poly...");
			cost = 391 * angle + 175;
			return cost;
		}
		/// Careful : the cost_function_poly is expressed as : 
		/// a_0 + a_1 x + ... + a_n x^n
		cost = cost_function_poly[cost_function_poly.size() - 1];  // Initialize result 
		for (int i = cost_function_poly.size() - 2; i >= 0; i--) // Horner method
			cost = cost * angle + cost_function_poly[i];
	} else {
		if(b)
			cost = 1 + angle * 0.4; // the heuristic should be tweaked according to application!!
		else
			cost = 391 * angle + 175; //Formula deduced from motors/Xplore.mlx
	}
	return cost;
}

float RAstarPlannerROS::getMoveCost(int i1, int j1, int i2, int j2){
	float moveCost=INFINIT_COST;//start cost with maximum value. Change it to real cost of cells are connected
	//if cell2(i2,j2) exists in the diagonal of cell1(i1,j1)
	if((j2==j1+1 && i2==i1+1)||(i2==i1-1 && j2==j1+1) ||(i2==i1-1 && j2==j1-1)||(j2==j1-1 && i2==i1+1)){
		//moveCost = DIAGONAL_MOVE_COST;
		moveCost = 1.41;
	} else{
		//if cell 2(i2,j2) exists in the horizontal or vertical line with cell1(i1,j1)
		if ((j2==j1 && i2==i1-1)||(i2==i1 && j2==j1-1)||(i2==i1+1 && j2==j1) ||(i1==i2 && j2==j1+1)){
			//moveCost = MOVE_COST;
			moveCost = 1;
		}
	}
	float h1 = OGM_continuous[getCellIndex(i1, j1)];
	float h2 = OGM_continuous[getCellIndex(i2, j2)];
	
	//float height_diff = h2 - h1;
	/**
	 * Max slope : 20°
	 * So we have y/x = 0.36 > 0.35
	 * */
	/*float max_elevation_per_meter = 0.35;
	float vertical_scale = 1;
	float threshold = max_elevation_per_meter * vertical_scale * resolution;
	if(abs(height_diff) > threshold) { // || h2 == nan_value) {
		//if(h2 == nan_value)
		//	cout << "nan value found : " << h2 << endl;
		return 1000000; //INFINIT_COST;
	}
	if(h2 == nan_value) {
		//cout << "nan value found : " << h2 << endl;
		return 800;
	}
	// TODO : remove this clause
	// if(height_diff < 0) return moveCost;
	// TODO : Test the cost function with : 'use_inferred_cost_function = true'
	float angle = atan(height_diff / (moveCost * resolution));
	float penalty = cost_function(max(angle, 0.0f));*/

	return moveCost * computePenalty(h1, h2, moveCost, false); //100;
}
 
float  RAstarPlannerROS::getMoveCost(int CellID1, int CellID2){
	int i1=0,i2=0,j1=0,j2=0;

	i1=getCellRowID(CellID1);
	j1=getCellColID(CellID1);
	i2=getCellRowID(CellID2);
	j2=getCellColID(CellID2);

	return getMoveCost(i1, j1, i2, j2);
} 

//verify if the cell(i,j) is free
bool  RAstarPlannerROS::isFree(int i, int j){
	//int CellID = getCellIndex(i, j);
	//return OGM[CellID];
	return true;
} 

//verify if the cell(i,j) is free
bool RAstarPlannerROS::isFree(int CellID){
	//return OGM[CellID];
	return true;
}

float RAstarPlannerROS::computePenalty(float h1, float h2, float moveCost, bool b) {	
	float height_diff = h2 - h1;
	float max_elevation_per_meter = 0.35;
	float threshold = max_elevation_per_meter * resolution;
	if(h2 == nan_value || h1 == nan_value) {
		return 400;
	}
	if(abs(height_diff) > threshold) { // || h2 == nan_value) {
		return 1000000;
	}
	
	float angle = atan(height_diff / (moveCost * resolution));
	float penalty = cost_function(max(angle, 0.0f), b);
	return penalty;
}

};

bool operator<(cells const &c1, cells const &c2) { return c1.fCost < c2.fCost; }

