/******************************************************************************
 *  MIT License
 *
 *  Copyright (c) 2019 Abhinav Modi 

 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *******************************************************************************/

/**@file walker.cpp
 * @brief Source file for walker algorithm for the Gazebo tutorial.
 *
 * Detailed description follows here.
 * @author     : Abhinav Modi
 * @created on : Nov 17, 2019
 */
#include "../include/walker.hpp"

Walker::Walker() {
    ROS_INFO_STREAM("turtlebot_walker node initialized");
    // initialize params
    isObstacle = false;
    // advertise velocity publisher
    velocityPub = nh.advertise<geometry_msgs::Twist>
                              ("/mobile_base/commands/velocity", 1000);
    // subscribe to latest scan data
    laserScanSub = nh.subscribe("/scan", 1000, &Walker::laserScanCb, this);

    // define initial velocities
    velMsg.linear.x = 0.0;
    velMsg.linear.y = 0.0;
    velMsg.linear.z = 0.0;
    velMsg.angular.x = 0.0;
    velMsg.angular.y = 0.0;
    velMsg.angular.z = 0.0;

    // publish the initial velocities
    velocityPub.publish(velMsg);
}

Walker::~Walker() {
    // Stop publishing velocities to stop the robot
    velMsg.linear.x = 0.0;
    velMsg.linear.y = 0.0;
    velMsg.linear.z = 0.0;
    velMsg.angular.x = 0.0;
    velMsg.angular.y = 0.0;
    velMsg.angular.z = 0.0;

    velocityPub.publish(velMsg);
}

void Walker::laserScanCb(const sensor_msgs::LaserScan::ConstPtr& velMsg) {
    for (int i=0; i < velMsg->ranges.size(); i++) {
        if (velMsg->ranges[i] < 0.70) {
            isObstacle = true;
            ROS_WARN_STREAM("Obstacle detected");
            return;
        }
    }
    isObstacle = false;
}

bool Walker::checkObstacle() {
    return isObstacle;  // returns true if an obstacle is detected
}

void Walker::moveRobot() {
    // initialize loop rate for the node
    ros::Rate loop_rate(10);
    // publish velocity commands till the node is running
    while (ros::ok()) {
        // if obstacle detected then stop and turn the turtlebot
        if (checkObstacle()) {
            ROS_INFO_STREAM("Obstacle ahead, changing direction");
            velMsg.linear.x = 0.0;
            velMsg.angular.z = -1.0;
        } else {
            // stop turning and keep moving forward
            ROS_INFO_STREAM("No obstacle detected, moving forward");
            velMsg.linear.x = 0.3;
            velMsg.angular.z = 0.0;
        }
        // publish the velocities
        velocityPub.publish(velMsg);

        ros::spinOnce();
        loop_rate.sleep();
    }
}
