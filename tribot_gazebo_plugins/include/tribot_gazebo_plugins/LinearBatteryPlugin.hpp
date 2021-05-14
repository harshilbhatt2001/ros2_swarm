#ifndef LINEAR_BATTERY_PLUGIN_HPP
#define LINEAR_BATTERY_PLUGIN_HPP

#include <string>
#include <map>

#include <sdf/sdf.hh>
#include "gazebo/common/Plugin.hh"
#include "gazebo/common/CommonTypes.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo_ros/node.hpp"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"

namespace gazebo {

class GazeboRosLinearBatteryPluginPrivate;
// A plugin that simulates a linear battery.
class GAZEBO_VISIBLE GazeboRosLinearBatteryPlugin : public ModelPlugin
{
public: 
    GazeboRosLinearBatteryPlugin();
    virtual ~GazeboRosLinearBatteryPlugin();
    virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
    virtual void Init();
    virtual void Reset();

private: 
    double OnUpdateVoltage(const common::BatteryPtr &_battery);
    void PublishBatteryState();

protected: 
    event::ConnectionPtr updateConnection;
    physics::WorldPtr world;
    physics::PhysicsEnginePtr physics;
    physics::ModelPtr model;
    physics::LinkPtr link;
    common::BatteryPtr battery;
    sdf::ElementPtr sdf;

/// \brief Open-circuit voltage.
/// E(t) = e0 + e1 * Q(t) / c
protected: double et;
protected: double e0;
protected: double e1;

/// \brief Initial battery charge in Ah.
protected: double q0;

/// \brief Battery capacity in Ah.
protected: double c;

/// \brief Battery inner resistance in Ohm.
protected: double r;

/// \brief Current low-pass filter characteristic time in seconds.
protected: double tau;

/// \brief Raw battery current in A.
protected: double iraw;

/// \brief Smoothed battery current in A.
protected: double ismooth;

/// \brief Instantaneous battery charge in Ah.
protected: double q;

/// \brief Charge Rate in A.
protected: double qt;

protected: bool charging;

protected: double sim_time_now;

private:
    std::unique_ptr<GazeboRosLinearBatteryPluginPrivate> impl_;
};


class GazeboRosLinearBatteryPluginPrivate
{
public:
    /// \brief Node for ros communication
    gazebo_ros::Node::SharedPtr ros_node_;
    /// \brief Publish for battery state message
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_state_pub_;
    /// \brief Battery state modified after each update
    sensor_msgs::msg::BatteryState::SharedPtr battery_state_msg_;
    
    


};

} // namespace gazebo

#endif /* LINEAR_BATTERY_PLUGIN_HPP */