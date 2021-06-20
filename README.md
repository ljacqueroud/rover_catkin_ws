# rover_catkin_ws

Repository containing various ROS packages needed for a path follower implementation in a rover simulation found [here](https://github.com/ljacqueroud/gym-gazebo). The packages are copied from [this repo](https://github.com/EPFLXplore/main_NAV_ws), which is the original repo of the rover simulation, but removing all the unused packages from it.

### Install the packages

Please note, required software needs to be installed first. Find all the information in the [main repo](https://github.com/ljacqueroud/gym-gazebo).\
To install the packages:
- clone this repo
- go to root: `cd rover_catkin_ws`
- `catkin_make`
- add `source catkin_ws/devel/setup.bash` to your `.bashrc` found in `/home/user`
