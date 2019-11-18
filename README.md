# turtlebot_walker
<a href='https://github.com/abhi1625/turtlebot_walker/blob/master/LICENSE'><img src='https://img.shields.io/badge/License-MIT-brightgreen.svg'/></a>

## Overview
A simple walker algorithm, like roomba, implemented on turtlebot. This project is a part of the gazebo turorial for ENPM808X - Software Development for robotics course.

## Dependencies
The following dependencies are required to run this package:

- ROS kinetic
- catkin
- Ubuntu 16.04
- Turtlebot packages
For installing ROS, follow the process given [here](http://wiki.ros.org/kinetic/Installation)

For installing catkin, follow the process given [here](http://wiki.ros.org/catkin#Installing_catkin)

For installing turtlebot packages, in a new terminal enter the following command:
```
sudo apt-get install ros-kinetic-turtlebot-*
```
This will install the turtlebot packages. 

**Note:** catkin is usually installed by default when ROS is installed.

## Building the package
To build this package follow the steps below:
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone --recursive https://github.com/abhi1625/turtlebot_walker.git
cd ..
catkin_make
source devel/setup.bash
```
## Running the demo
Once the build is complete successfully, you can run it using roslaunch:
```
roslaunch turtlebot_walker turtlebot_walker.launch
```
This will start the gazebo simulation with the turtlebot in it as well as the `walker` node. If needed, you can also run both nodes separately using:
```
roscore
```
Then in a fresh terminal launch the gazebo simulation with a turtlebot:
```
roslaunch turtlebot_gazebo turtlebot_world.launch
```
Finally, run the `walker` node using:
```
rosrun turtlebot_walker turtlebot_walker
```
## Recording bag files
To enable recording of all topics, you can use the argument `record:=true`(By default it is set as false).
```
roslaunch turtlebot_walker turtlebot_walker.launch record:=true
```
You can also specify the time for which you want to record the bag file using:
```
roslaunch turtlebot_walker turtlebot_walker.launch record:=true record_time:=30
```
This will record the bag file for 30s and save it in the results subdirectory.

To examine the recorded bag file, use:
```
rosbag info results/turtlebot_walker.bag
```
## Playing the bag file
To inspect the recorded bag file, go to the results subdirectory and enter:
```
rosbag info turtlebot_walker.bag
```
This will give the information about all the topics recorded and duration of recording of the bag file. To play the bag file use:
```
rosbag play turtlebot_walker.bag
```
Now, you can view the recorded messages being published on the topic `/mobile_base/commands/velocity` as:
```
rostopic echo /mobile_base/commands/velocity
```