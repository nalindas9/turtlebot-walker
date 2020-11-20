# turtlebot-walker
[![License:MIT](https://img.shields.io/badge/License-MIT-green.svg)](https://github.com/nalindas9/turtlebot_walker/blob/Week12_HW/LICENSE)

Walker Algorithm implementation for the Turtlebot 3 in Gazebo

## Overview

This ROS package implements a walker algorithm (similar to a Roomba robot vacuum cleaner) for the Turtlebot 3 in Gazebo. The robot moves forward until it reaches an obstacle (but not colliding), then rotates in place until the way ahead is clear, then moves forward again and repeats.

## Dependencies

The following dependencies are required to run this package:

1. ROS Melodic
2. catkin 
3. Ubuntu 18.04 

## Standard install via command-line
```
cd ~/catkin_ws/src
git clone --recursive https://github.com/nalindas9/turtlebot_walker.git
cd ..
catkin_make
```
## Running Simulation
Open the following terminals and run the following commands in them:

1. Terminal 1:
```
roscore
```

2. Terminal 2:
Passing record=true for recording of bag file:
```
cd catkin_ws
source devel/setup.bash
roslaunch turtlebot_walker turtlebot_walker.launch 
```
## Inspecting rosbag file
1. Terminal 1:
```
cd catkin_ws
source devel/setup.bash
cd src/turtlebot_walker/results/
rosbag info turtlebot_walker.bag
```

## Playing the rosbag file
1. Terminal 1 (run master node):
```
roscore
```

2. Terminal 2 (play rosbag):
```
cd catkin_ws
source devel/setup.bash
rosbag play src/turtlebot_walker/results/turtlebot_walker.bag
```
