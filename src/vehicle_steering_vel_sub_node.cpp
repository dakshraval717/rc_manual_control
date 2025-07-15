#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <cmath>

#define TURNING_RADIOUS_MIN 0.35
#define VEHICLE_D 0.1667
#define VEHICLE_L 0.1778

ros::Publisher vehicle_twist_pub;

void vehicle_steering_vel_callback(const geometry_msgs::Twist::ConstPtr& steering_vel_inputs) {
  double turning_radius = VEHICLE_L / tan(steering_vel_inputs->angular.z);
  double vx_by_vL = steering_vel_inputs->linear.x / ((turning_radius - VEHICLE_D/2) / turning_radius);
  double vx_by_vR = steering_vel_inputs->linear.y / ((turning_radius + VEHICLE_D/2) / turning_radius);
  double vx = (vx_by_vL + vx_by_vR) / 2;
  double yaw_rate = vx / turning_radius;
  
  geometry_msgs::Twist vehicle_twist;
  vehicle_twist.linear.x = vx;
  vehicle_twist.angular.z = yaw_rate;
  vehicle_twist_pub.publish(vehicle_twist);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "steering_vel_sub_node");
  ros::NodeHandle nh;

  ros::Subscriber vehicle_steering_vel_sub = nh.subscribe<geometry_msgs::Twist>("/vehicle_current_steering_vel", 10, vehicle_steering_vel_callback);
  vehicle_twist_pub = nh.advertise<geometry_msgs::Twist>("/vehicle_current_twist", 1);

  ros::spin();
  return 0;
}

