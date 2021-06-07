#include <tribot_gazebo_plugins/GazeboRosLinearBatteryPlugin.hpp>

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

namespace gazebo
{

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

void GazeboRosLinearBatteryPlugin::Load(physics::ModelPtr model, sdf::ElementPtr _sdf)
{
    const gazebo_ros::QoS &qos = impl_->ros_node_->get_qos();


    if (_sdf->HasElement("namespace"))
    {
        impl_->robot_namespace_ = _sdf->Get<std::string>("namespace");
    }

    // Update rate
      double update_rate = 100.0;
    if (!sdf->HasElement("update_rate")) 
    {
        RCLCPP_INFO(impl_->ros_node_->get_logger(), "Missing <update_rate>, defaults to %f", update_rate);
    } else {
        update_rate = sdf->GetElement("update_rate")->Get<double>("update_rate");
    }

    if (update_rate > 0.0) {
        impl_->update_period_ = 1.0 / update_rate;
    } else {
        impl_->update_period_ = 0.0;
    }

    std::string link_name = _sdf->Get<std::string>("link_name");
    this->link = this->model->GetLink(link_name);

    this->e0    = _sdf->Get<double>("open_circuit_voltage_constant_coef");
    this->e1    = _sdf->Get<double>("open_circuit_voltage_linear_coef");
    this->q0    = _sdf->Get<double>("initial_charge");
    //this->qt    = _sdf->Get<double>("charge_rate");
    this->c     = _sdf->Get<double>("capacity");
    this->r     = _sdf->Get<double>("resistance");
    this->tau   = _sdf->Get<double>("smooth_current_tau");

    std::string battery_name = _sdf->Get<std::string>("battery_name");
    
    // Create battery
    if (this->link->BatteryCount() > 0)
    {
        this->battery = this->link->Battery(battery_name);
    }


    impl_->last_update_time_ = model->GetWorld()->SimTime();

    /*
    // Battery State Publisher
    impl_->battery_state_pub_ = impl_->ros_node_->create_publisher<sensor_msgs::msg::BatteryState>(
        "battery_state", qos.get_publisher_qos("battery_state", rclcpp::QoS(1000)));
    
    // callback on every iteration
    impl_->update_connection_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&GazeboRosLinearBatteryPlugin::PublishBatteryState, impl_.get(), std::placeholders::_1));
    */
   // Specify custom Update function
   this->battery->SetUpdateFunc(std::bind(&GazeboRosLinearBatteryPlugin::OnUpdateVoltage, this, std::placeholders::_1));
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

    this->sim_time_now = this->world->SimTime();

    this->et = this->e0 + this->e1 * (1 - this->q / this->c) - this->r * this->ismooth;

    // TODO: Turn off bot when battery is 0 
    // Turn off the motor
    if (this->q <= 0)
    {
        this->sim_time_now = this->world->SimTime();
        this->q = 0;
        //TODO: Motor control 
    }
    else if (this->q >= this->c)
    {
        this->q = this-> c;
    }


    // Negative sim time (when world is simulated)
    if (this->sim_time_now < impl_->last_update_time_)
    {
        RCLCPP_INFO(impl_->ros_node_->get_logger(), "Negative sim time difference detected.");
        impl_->last_update_time_ = this->sim_time_now;
    }

    // Check period
    // double seconds_since_last_update = (this->sim_time_now - impl_->last_update_time_).Double();
    // if (seconds_since_last_update < impl_->update_period_)
    // {
    //     return;
    // }

    // Populate Message
    sensor_msgs::msg::BatteryState battery_state;
    battery_state.header.stamp      = rclcpp::Clock(RCL_STEADY_TIME).now();
    battery_state.header.frame_id   = this->link->GetName();
    battery_state.charge            = this->q;
    battery_state.percentage        = this->q / this->q0;
    battery_state.voltage           = this->battery->Voltage();
    battery_state.design_capacity   = this->q0;
    //TODO: Add mode data

    impl_->battery_state_pub_->publish(battery_state);

    // update time
    impl_->last_update_time_ = this->sim_time_now;

    return et;

}

void GazeboRosLinearBatteryPlugin::PublishBatteryState(const common::UpdateInfo &info)
{
}
GZ_REGISTER_MODEL_PLUGIN(GazeboRosLinearBatteryPlugin)
} // namespace gazebo