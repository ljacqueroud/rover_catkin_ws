# XploreNavigation

ROS workspace implemented to carry out the autonomous navigation task

## Usage
* To visualize the rover in rViz run: 
```bash
roslaunch rover_description display.launch 
```
* To start Gazebo, ros_control and spawn the rover run:
```bash
roslaunch rover_description gazebo.launch
```
In rViz, the robot model and the reference frame should be manually selected

Once Gazebo is running, to command a wheel to spin type in a new terminal:
```bash
rostopic pub -1 /rover/<wheel_joint_name>/command std_msgs/Float64 "data: <speed_value>"
```

## Dependencies

ros_control requires the following packages:
* ros_controllers (clone from https://github.com/ros-controls/ros_controllers.git , melodic devel branch)
* ros_control (clone from https://github.com/ros-controls/ros_control.git , melodic devel branch)

Addionally, you also need:
* joint_state_publisher
* robot_state_publisher
* urdf_geometry_parser

Check with ```rospack find <package_name>``` or clone/install from github

## Authors
The Xplore Navigation team

