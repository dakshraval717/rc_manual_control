#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <cmath>
#include "string.h"
#include "stdlib.h"

#define LINEAR_VEL_AXES 1
#define ANGULAR_VEL_AXES 3
#define ENABLE_BUTTON 4
#define TURNING_RADIOUS_MIN 0.35
#define VEHICLE_D 0.1667
#define VEHICLE_L 0.1778
#define STEERING_ANGLE_MAX 0.47
#define MOTOR_LINEAR_V_MAX 1.0

class joy_vel_publisher {
public:
    joy_vel_publisher(ros::NodeHandle& nh, int cmd_rate) {
	vel_cmd_rate = cmd_rate;
	
	joy_sub = nh.subscribe("/joy", 1, &joy_vel_publisher::joyCallback, this);
	steering_vel_pub = nh.advertise<geometry_msgs::Twist>("vehicle_target_steering_vel", 1);
    }

    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {
        motion_enable = joy->buttons[ENABLE_BUTTON];
        steering_angle_output = STEERING_ANGLE_MAX * joy->axes[ANGULAR_VEL_AXES];
        
        if(steering_angle_output == 0) {
          v_left_output = v_right_output = MOTOR_LINEAR_V_MAX * joy->axes[LINEAR_VEL_AXES];
        }
        else {
          double turning_radius = VEHICLE_L / tan(steering_angle_output);
          
          if(turning_radius > 0) {
            v_right_output = MOTOR_LINEAR_V_MAX * joy->axes[LINEAR_VEL_AXES];
            v_left_output = (turning_radius - VEHICLE_D/2) / (turning_radius + VEHICLE_D/2) * v_right_output;
          }
          else {
            v_left_output = MOTOR_LINEAR_V_MAX * joy->axes[LINEAR_VEL_AXES];
            v_right_output = (turning_radius + VEHICLE_D/2) / (turning_radius - VEHICLE_D/2) * v_left_output;            
          }
        }
    }

    void velUpdateLoop() {
        ros::Rate loop_rate(vel_cmd_rate);
        geometry_msgs::Twist steering_vel_cmd;

        while (ros::ok()) {
            if(motion_enable) {
            	steering_vel_cmd.linear.x = v_left_output;
            	steering_vel_cmd.linear.y = v_right_output;
            	steering_vel_cmd.angular.z = steering_angle_output;
            }
            else {
            	steering_vel_cmd.linear.x = 0.0;
            	steering_vel_cmd.linear.y = 0.0;
            	steering_vel_cmd.angular.z = 0.0;
            }
        
            steering_vel_pub.publish(steering_vel_cmd);
            ros::spinOnce();
            loop_rate.sleep();
        }
    }

private:
    ros::Publisher steering_vel_pub;
    ros::Subscriber joy_sub;

    int motion_enable;
    int vel_cmd_rate;
    double steering_angle_output;
    double v_left_output;
    double v_right_output;
};

int main(int argc, char **argv)
{
 ros::init(argc, argv, "steering_vel_pub_node");
 ros::NodeHandle nh;
 
 if (argc < 2) {
  ROS_ERROR("Run the steering velocity command node with the mode argument. Exit.");
  return 0;
 }

 if (strcmp(argv[1], "manual") == 0) {
  joy_vel_publisher joystick_steering_vel_cmd(nh, 20);
  joystick_steering_vel_cmd.velUpdateLoop();
 }
 else if (strcmp(argv[1], "automatic") == 0) {
  // Your work :)
 }
 else {
  ROS_ERROR("Wrong mode argument entered. Exit.");
  return 0;
 }

 return 0;
}

