#include <iostream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "turtlebot_walker.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "walker");
ros::NodeHandle nh;
   Walker walk(nh);
  return 0;
}
