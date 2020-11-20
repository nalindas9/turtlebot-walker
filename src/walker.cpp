/**
 *  Copyright 2020 Nalin Das
 *  @file walker.cpp
 *  @author Nalin Das
 *  @date 11/18/2020
 *
 *  @brief Walker class methods implementation
 *
 *  @section LICENSE
 *  
 * MIT License
 * Copyright (c) 2020 Nalin Das
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
 * 
 *  @section DESCRIPTION
 *
 *  Source file for Walker Class methods
 *
 */

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include "geometry_msgs/Twist.h"
#include "walker.hpp"

void sdw::Walker::laserCallback(const sensor_msgs::LaserScan::ConstPtr& data) {
    distance = data->ranges[180];
    ROS_INFO_STREAM("Distance: " << distance);
}

sdw::Walker::Walker(ros::NodeHandle node) {
    // ROS subscriber to LaserScan
    ros::Subscriber laserSubscriber = node.subscribe("/scan", 1000, &Walker::laserCallback, this);

    // ROS publisher to velocity topic
    ros::Publisher velocityPublisher = node.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    // Looprate of 4 Hz
    ros::Rate rate(4);

    while (ros::ok()) {
        // Define twist msg
        geometry_msgs::Twist twist;
        // Initialize to all zeros
        twist.linear.x = 0.0;
        twist.linear.y = 0.0;
        twist.linear.z = 0.0;
        twist.angular.x = 0.0;
        twist.angular.y = 0.0;
        twist.angular.z = 0.0;

        if (distance > 0.45) {
            ROS_INFO_STREAM("Moving forward ...");
            twist.linear.x = -0.12;
        } else {
            ROS_INFO_STREAM("Rotating ...");
            twist.angular.z = 1.5;
        }

        velocityPublisher.publish(twist);
        ros::spinOnce();
        rate.sleep();
    }
}
