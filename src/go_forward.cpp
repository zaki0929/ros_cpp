#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

#define RANGE_MAX 5.6

double center = 0;
double left = 0;
double right = 0;

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
  center = msg->ranges[center_number];
  left = msg->ranges[center_number+255];
  right = msg->ranges[center_number-255];

  center = null_check(center);
  left = null_check(left);
  right = null_check(right);

  ROS_INFO("center: [%lf], left: [%lf], right: [%lf]", center, left, right);
  //ROS_INFO("center number: [%lf]", (-msg->angle_min)/msg->angle_increment);

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "go_forward");
  ros::NodeHandle n;
  ros::Subscriber scan_sub = n.subscribe("scan", 1000, scanCallback);
  ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  geometry_msgs::Twist cmd_vel;

  if(center < 0.3){
    ROS_WARN("center warning!!");
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.2;
  }
  if(left < 0.3){
    ROS_WARN("left warning!!");
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = -0.2;
  }
  if(right < 0.3){
    ROS_WARN("right warning!!");
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.2;
  }
  if(center >=0.3 && left >= 0.3 && right >= 0.3){
    ROS_WARN("right warning!!");
    cmd_vel.linear.x = 0.2;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.0;
  }

  vel_pub.publish(cmd_vel);

  ros::spin();

  return 0;
}

