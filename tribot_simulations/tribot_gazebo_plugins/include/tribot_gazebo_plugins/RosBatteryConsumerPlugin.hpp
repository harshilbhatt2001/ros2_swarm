#ifndef ROS_BATTERY_CONSUMER_PLUGIN_HPP
#define ROS_BATTERY_CONSUMER_PLUGIN_HPP

#include <string>
#include <map>

#include <sdf/sdf.hh>
#include "gazebo/common/Plugin.hh"
#include "gazebo/common/CommonTypes.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo_ros/node.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

namespace gazebo 
{

class RosBatteryConsumerPlugin : public ModelPlugin 
{
public:
    RosBatteryConsumerPlugin();
    virtual ~RosBatteryConsumerPlugin();
    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

protected:
    void UpdateDeviceState(const std_msgs::msg::Bool &_msg);
    void UpdatePowerLoad(double _powerload = 0.0);

protected:
    event::ConnectionPtr updateConnection;
    physics::WorldPtr world;
    physics::PhysicsEnginePtr physics;
    physics::ModelPtr model;
    physics::LinkPtr link;
    common::BatteryPtr battery;
    sdf::ElementPtr sdf;

protected:
    /// \brief Node for ros communication
    gazebo_ros::Node::SharedPtr ros_node_;
    /// \brief Subscriber to device state flag
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr device_state_sub_;
    /// \brief Flag to signal if specific device is running
    bool is_device_on_;
    /// \brief Power load in W
    double power_load_;
    /// \brief Battery consumer ID
    int consumer_id_;
    /// \brief Link name
    std::string link_name_;
    /// \brief battery model name
    std::string battery_name_;

};

} // namespace gazebo

#endif /* ROS_BATTERY_CONSUMER_PLUGIN_HPP */