#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"


class Walker {
 public:
    Walker(ros::NodeHandle& nh);
    void callback(const sensor_msgs::LaserScan::ConstPtr& readings);
    ~Walker();
 private:
    double dist;
    ros::Subscriber sub;
    ros::Publisher pub;
    geometry_msgs::Twist msg;
};
