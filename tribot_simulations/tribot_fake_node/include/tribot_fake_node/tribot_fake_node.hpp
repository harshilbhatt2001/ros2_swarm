#ifndef TRIBOT_FAKE_NODE_HPP
#define TRIBOT_FAKE_NODE_HPP

#include <tf2/LinearMath/Quaternion.h>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#define LEFT 0
#define RIGHT 1

class TribotFake : public rclcpp::Node
{
public:
    TribotFake();
    ~TribotFake();

private:
    ///\brief ROS time
    rclcpp::Time last_cmd_vel_time_;
    rclcpp::Time prev_update_time_;

    ///\brief ROS timer
    rclcpp::TimerBase::SharedPtr update_timer_;

    ///\brief ROS publishers
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub_;
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_pub_;

    ///\brief ROS subscribers
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    nav_msgs::msg::Odometry odom_;
    sensor_msgs::msg::JointState joint_states_;

    double wheel_speed_cmd_[2];
    double goal_linear_velocity_;
    double goal_angular_velocity_;
    double cmd_vel_timeout_;
    double last_position_[2];
    double last_velocity_[2];
    float odom_pose_[3];
    float odom_vel_[3];

    double wheel_seperation_;
    double wheel_radius_;

    ///\brief Function prototypes
    void init_parameters();
    void init_variables();
    void command_velocity_callback(const geometry_msgs::msg::Twist::SharedPtr cmd_vel_msg);
    void update_callback();
    bool update_odometry(const rclcpp::Duration & diff_time);
    void update_joint_state();
    void update_tf(geometry_msgs::msg::TransformStamped & odom_tf);

};

#endif /* TRIBOT_FAKE_NODE_HPP */