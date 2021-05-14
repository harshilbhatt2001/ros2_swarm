#include <tribot_gazebo_plugins/LinearBatteryPlugin.hpp>

#include <sdf/sdf.hh>

#include <gazebo/plugins/LinearBatteryPlugin.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/common/Plugin.hh>
#include "gazebo_ros/node.hpp"
#include <gazebo/common/Battery.hh>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"

enum power {
    OFF = 0,
    ON = 1
};

using namespace gazebo;

GazeboRosLinearBatteryPlugin::GazeboRosLinearBatteryPlugin()
  : impl_(std::make_unique<GazeboRosLinearBatteryPluginPrivate>())
{
    this->c = 0.0;
    this->r = 0.0;
    this->tau = 0.0;
    this->e0 = 0.0;
    this->e1 = 0.0;
    this->q0 = 0.0;
    this->q = 0.0;
    this->ismooth = 0.0;
    this->iraw = 0.0;
}


GazeboRosLinearBatteryPlugin::~GazeboRosLinearBatteryPlugin()
{
}
//TODO
void GazeboRosLinearBatteryPlugin::Load(physics::ModelPtr, sdf::ElementPtr sdf)
{
    impl_->ros_node_ = gazebo_ros::Node::Get(sdf);


}

void GazeboRosLinearBatteryPlugin::Init()
{
    this->q = this->q0;
    this->charging = false;
}

void GazeboRosLinearBatteryPlugin::Reset()
{
    this->iraw = 0.0;
    this->ismooth = 0.0;
    this->Init();
}

double GazeboRosLinearBatteryPlugin::OnUpdateVoltage(const common::BatteryPtr &_battery)
{
    double dt = this->world->Physics()->GetMaxStepSize();
    double totalpower = 0.0;
    double k = dt / this->tau;

    if (fabs(_battery->Voltage()) < 1e-3)
    {
        return 0.0;
    }

    for (auto powerLoad : _battery->PowerLoads())
    {
        totalpower += powerLoad.second;
    }

    this->iraw = totalpower / _battery->Voltage();
    this->ismooth = this->ismooth * k * (this->iraw - this->ismooth);

    if (!this->charging)
    {
        this->q = this->q - GZ_SEC_TO_HOUR(dt * this->ismooth);
    } else {
        this->q = this->q + GZ_SEC_TO_HOUR(dt + this->qt);
    }

    this->sim_time_now = this->world->SimTime().Double();

    this->et = this->e0 + this->e1 * (1 - this->q / this->c) - this->r * this->ismooth;

    // TODO: Turn off bot when battery is 0 
}

void GazeboRosLinearBatteryPlugin::PublishBatteryState()
{
    impl_->battery_state_msg_->header.stamp = rclcpp::Clock(RCL_STEADY_TIME).now();
    impl_->battery_state_msg_->header.frame_id = this->link->GetName();

    impl_->battery_state_msg_->charge = this->q;
    impl_->battery_state_msg_->percentage = this->q / this->q0;
    impl_->battery_state_msg_->voltage = this->battery->Voltage();
    impl_->battery_state_msg_->design_capacity = this->q0;

    //TODO: Add mode data

    const gazebo_ros::QoS &qos = impl_->ros_node_->get_qos();
    impl_->battery_state_pub_ = impl_->ros_node_->create_publisher<sensor_msgs::msg::BatteryState>(
        "battery_state", qos.get_publisher_qos("battery_state", rclcpp::QoS(1000)));
}