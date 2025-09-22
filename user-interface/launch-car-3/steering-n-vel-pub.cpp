// rc_car_data_publisher.cpp
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>

class RCCarDataPublisher {
private:
    ros::NodeHandle nh_;
    ros::Publisher steering_pub_;
    ros::Publisher wheel_vel_pub_;
    
    // ESP32 communication
    ros::Subscriber esp32_data_sub_;
    
public:
    RCCarDataPublisher() {
        // Publishers for plotting
        imu_pub_ = nh_.advertise<sensor_msgs::Imu>("imu/data", 10);
        steering_pub_ = nh_.advertise<std_msgs::Float32>("steering_angle", 10);
        wheel_vel_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("wheel_velocities", 10);
        
        // Subscriber for ESP32 data
        esp32_data_sub_ = nh_.subscribe("esp32/sensor_data", 10, 
                                       &RCCarDataPublisher::esp32DataCallback, this);
        
        ROS_INFO("RC Car Data Publisher initialized");
    }
    
    void esp32DataCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
        // Parse ESP32 data and publish to individual topics
        publishIMUData(msg->data[0], msg->data[1]); // accel x, y
        publishSteeringAngle(msg->data[2]);
        publishWheelVelocities(msg->data[3], msg->data[4]); // left, right
    }
    
    void publishIMUData(float accel_x, float accel_y) {
        sensor_msgs::Imu imu_msg;
        imu_msg.header.stamp = ros::Time::now();
        imu_msg.linear_acceleration.x = accel_x;
        imu_msg.linear_acceleration.y = accel_y;
        imu_pub_.publish(imu_msg);
    }
    
    void publishSteeringAngle(float angle) {
        std_msgs::Float32 steering_msg;
        steering_msg.data = angle;
        steering_pub_.publish(steering_msg);
    }
    
    void publishWheelVelocities(float left_vel, float right_vel) {
        std_msgs::Float32MultiArray vel_msg;
        vel_msg.data.push_back(left_vel);
        vel_msg.data.push_back(right_vel);
        wheel_vel_pub_.publish(vel_msg);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "rc_car_data_publisher");
    RCCarDataPublisher publisher;
    ros::spin();
    return 0;
}
