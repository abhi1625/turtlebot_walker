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

/**@file walker.hpp
 * @brief Header file for walker algorithm node for the Gazebo tutorial
 *
 * Detailed description follows here.
 * @author     : Abhinav Modi
 * @created on : Nov 17, 2019
 */

#ifndef INCLUDE_WALKER_HPP_
#define INCLUDE_WALKER_HPP_

// ROS headers
#include <geometry_msgs/Twist.h>
#include <iostream>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
// CPP headers

/**
 *  @brief Class Walker:
 *  The following class subscribes to laserScan messages and  publishes
 *  command velocity to turtlebot to illustrate a simple navigation algorithm  
 */
class Walker {
 private:
    /**
     * @brief       Node handler for walker
     */
    ros::NodeHandle nh;
    /**
     * @brief       Subscriber for the laserscan topic
     */
    ros::Subscriber laserScanSub;
    /**
     * @brief       Velocity publisher
     */    
    ros::Publisher velocityPub;

    bool isObstacle;              // boolean to detect if there is a collision
    geometry_msgs::Twist velMsg;  // Variable to detect

 public:
    /**
     * @brief  Default constructor for Walker class  
     * @param  none
     * @return void
     */
    Walker();
    /**
     * @brief  Default destructor for Walker class  
     * @param  none
     * @return void
     */    
    ~Walker();

    /**
     * @brief  Callback function for subscribing to laserScan data  
     * @param  pointer to LaserScan message
     * @return void
     */    
    void laserScanCb(const sensor_msgs::LaserScan::ConstPtr& velMsg);

    /**
     * @brief  moveRobot publishes the velocity to turtlebot to move it around  
     * @param  none
     * @return void
     */
    void moveRobot();

    /**
     * @brief  function to check if there is an obstacle  
     * @param  none
     * @return bool - true if an obstacle is detected else false
     */    
    bool checkObstacle();
};

#endif  // INCLUDE_WALKER_HPP_
