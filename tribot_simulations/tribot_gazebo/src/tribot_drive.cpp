#include "tribot_gazebo/include/tribot_drive.hpp"

using namespace std::chrono_literals;

TribotDrive::TribotDrive()
  : Node("tribot_drive_node") 
{
    /// init variables
    robot_pose_ = 0.0;
    prev_robot_pose_ = 0.0;

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", qos);
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", qos, std::bind(&TribotDrive::odom_callback, this, std::placeholders::_1));
    
    /// init timer
    update_timer_ = this->create_wall_timer(10ms, std::bind(TribotDrive::update_callback, this));
}

TribotDrive::~TribotDrive()
{
}

void TribotDrive::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    robot_pose_ = yaw;
}

void TribotDrive::update_cmd_vel(double linear, double angular)
{
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = linear;
    cmd_vel.angular.z = angular;
    
    cmd_vel_pub_->publish(cmd_vel);
}

void TribotDrive::update_callback()
{
    //TODO
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TribotDrive>());
    rclcpp::shutdown();

    return 0;
}