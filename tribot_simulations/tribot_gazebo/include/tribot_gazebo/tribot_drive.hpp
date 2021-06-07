#ifndef TRIBOT_DRIVE_HPP
#define TRIBOT_DRIVE_HPP

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <rclcpp/rclcpp.hpp>


#define DEG2RAD (M_PI / 180.0)
#define RAD2DEG (180.0 / M_PI)

#define CENTER 0
#define LEFT 1
#define RIGHT 2

#define LINEAR_VELOCITY 0.3
#define ANGULAR_VELOCITY 1.5

#define GET_TRIBOT_DIRECTION 0
#define TRIBOT_DRIVE_FORWARD 1
#define TRIBOT_RIGHT_TURN 2
#define TRIBOT_LEFT_TURN 3

class TribotDrive : public rclcpp::Node 
{
public:
    TribotDrive();
    ~TribotDrive();
private:
    ///\brief: ROS topic publisher
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

    ///\brief: ROS topic subscriber
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    ///\brief: variables
    double robot_pose_;
    double prev_robot_pose_;

    ///\brief: ROS timer 
    rclcpp::TimerBase::SharedPtr update_timer_;

    ///\brief prototypes
    void update_callback();
    void update_cmd_vel(double linear, double angular);
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
};

#endif /* TRIBOT_DRIVE_HPP */