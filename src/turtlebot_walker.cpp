/**
 *  @file    turtlebot_walker.cpp
 *  @author  Pranav Dhulipala
 *  @copyright  MIT License (c) 2017 Pranav Dhulipala
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 *  The above copyright notice and this permission notice shall be included in all
 *  copies or substantial portions of the Software.
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *  SOFTWARE.
 *
 * @brief contains the definitions used in turtlebot_walker.h
 */
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "turtlebot_walker.h"  // NOLINT
/**
 * @brief      Callback for the subscriber
 *
 * @param  readings  Message received over /scan topic
 */
void Walker::callback(const sensor_msgs::LaserScan::ConstPtr& readings) {
  double min = 0;
  for (int i = 0; i < readings->ranges.size(); i++) {
    if (readings->ranges[i] > min)
      min = readings->ranges[i];
  }
  // stores the minimum of the laser readings
  dist = min;
  ROS_INFO_STREAM("Distance Ahead " << dist);
}
/**
 * @brief      Constructor to create object 
 *              and generates the velocities
 * @param      nh     Nodehandle
 */
Walker::Walker(ros::NodeHandle& nh) {
  sub = nh.subscribe("/scan", 1000, &Walker::callback, this);
  pub = nh.advertise < geometry_msgs::Twist
      > ("/mobile_base/commands/velocity", 1);
  ros::Rate loop_rate(2);
  while (nh.ok()) {
    msg.linear.x = 0.0;
    msg.angular.z = 0.0;
    if (dist > 0.8) {
      msg.linear.x = 0.1;
      ROS_INFO_STREAM("To infinity and beyond");
    } else {
      msg.angular.z = 1.5;
      ROS_WARN_STREAM("Obstacle ahead, turning");
    }
    pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
}
/**
 * @brief      Destructor stops the turtlebot at exit
 *             
 */
Walker::~Walker() {
  msg.linear.x = 0.0;
  msg.angular.z = 0.0;
  pub.publish(msg);
}
