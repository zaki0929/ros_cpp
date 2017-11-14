#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

#define RANGE_MAX 5.6

double null_check(double target){
  if(!(target > 0)){
    target = (double)RANGE_MAX;
    ROS_WARN("RANGE OVER");
  }
  return target;
}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  double center_number = (-msg->angle_min)/msg->angle_increment;
  double center = msg->ranges[center_number];
  double left = msg->ranges[center_number+255];
  double right = msg->ranges[center_number-255];

  center = null_check(center);
  left = null_check(left);
  right = null_check(right);

  ROS_INFO("center: [%lf], left: [%lf], right: [%lf]", center, left, right);
  //ROS_INFO("center number: [%lf]", (-msg->angle_min)/msg->angle_increment);

  if(center < 0.3){
    ROS_WARN("center warning!!");
  }
  if(left < 0.3){
    ROS_WARN("left warning!!");
  }
  if(right < 0.3){
    ROS_WARN("right warning!!");
  }

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "urg_sub");
  ros::NodeHandle n;
  ros::Subscriber scan_sub = n.subscribe("scan", 1000, scanCallback);
  ros::spin();

  return 0;
}

