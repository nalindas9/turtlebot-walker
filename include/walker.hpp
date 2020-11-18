#pragma once
/**
 *  Copyright 2020 Nalin Das
 *  @file walker.hpp
 *  @author Nalin Das
 *  @date 11/18/2020
 *
 *  @brief Walker class 
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
 *  Header file for Walker Algorithm Class 
 *
 */

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>

namespace sdw {
/**
 * @brief Walker class
 * **/
class Walker {
 private:
    /**
     * @brief Stores distance (in meters) from obstacle.
     * **/
    double distance;
 public:
    /**
     * @brief Walker class constructor.
     * @param node ROS Nodehandle
     * **/
    Walker(ros::NodeHandle node);
    
    /**
     * @brief Laserscan subscriber callback
     * @param data Single scan from a planar laser range finder
     * **/
    void laser_callback(const sensor_msgs::LaserScan::ConstPtr& data);

    /**
     * @brief Walker class destructor.
     * @param node ROS Nodehandle
     * **/
    ~Walker(ros::NodeHandle node) {};
};
} // namespace sdw
