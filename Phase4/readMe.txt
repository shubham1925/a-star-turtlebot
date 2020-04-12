## Authors

Raj Prakash Shinde - 116852104
Prasheel Renkuntla - 116925570
Shubham Sonawane - 116808996

## Description
It is the same A star to run on the Turtlebot.
Originaly being developed on Wheelchair but due to timeline, submitting the Turtlebot Version

## Dependencies
1. Ubuntu 16.04
2. C++ 11/14/17
3. ROS Kinetic- http://wiki.ros.org/kinetic/Installation
4. Turtlebot stack
Install by running following command
	sudo apt-get install ros-kinetic-turtlebot-gazebo ros-kinetic-turtlebot-apps ros-kinetic-turtlebot-rviz-launchers
5. GMapping
6. amcl
7. Navigation stack

## Build
Steps to build

	mkdir -p ~/catkin_ws/src
	cd ~/catkin_ws/
	catkin_make
	source devel/setup.bash
	cd src/
	git clone https://github.com/shubham1925/a-star-turtlebot
	cd ~/catkin_ws/
	catkin_make


## Run
Open a new Terminal
	source devel/setup.bash
	roslaunch pathtracer trace.launch

Open a new Terminal
	source devel/setup.bash
	rosrun pathtracer tracer

## Bugs
ecobot sometimes take more time to collect/delete garbage even after reaching near, because A* path planner takes more time to achieve exact target location.
