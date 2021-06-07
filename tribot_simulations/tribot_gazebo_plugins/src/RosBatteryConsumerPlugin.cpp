#include <tribot_gazebo_plugins/RosBatteryConsumerPlugin.hpp>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/common/Battery.hh>

namespace gazebo 
{
RosBatteryConsumerPlugin::RosBatteryConsumerPlugin()
{
    this->is_device_on_ = true;
}

RosBatteryConsumerPlugin::~RosBatteryConsumerPlugin()
{
}

void RosBatteryConsumerPlugin::Load(physics::ModelPtr model, sdf::ElementPtr _sdf)
{
    this->ros_node_.reset();

    this->model = model;


    this->link_name_ = _sdf->Get<std::string>("link_name");
    this->link = this->model->GetLink(this->link_name_);
    
    // create battery
    this->battery_name_ = _sdf->Get<std::string>("battery_name");
    this->battery = this->link->Battery(this->battery_name_);

    // add consumer and set its power load
    // this->power_load_ = _sdf->Get<double>("power_load");
    // this->consumer_id_ = this->battery->AddConsumer();
    // this->battery->SetPowerLoad(this->consumer_id_, this->power_load_);

    if (_sdf->HasElement("topic_device_state"))
    {
        std::string topicName = _sdf->Get<std::string>("topic_device_state");
        if (!topicName.empty())
        {
            this->device_state_sub_ = this->ros_node_->create_subscription<std_msgs::msg::Bool>(
                topicName, 1, std::bind(&RosBatteryConsumerPlugin::UpdateDeviceState, this, std::placeholders::_1));
        }
    } else {
        this->UpdatePowerLoad(this->power_load_);
    }
}

void RosBatteryConsumerPlugin::UpdateDeviceState(const std_msgs::msg::Bool &_msg)
{
    this->is_device_on_ = _msg.data;
    if (this->is_device_on_)
    {
        this->UpdatePowerLoad(this->power_load_);
    } else {
        this->UpdatePowerLoad(0.0);
    }
}

void RosBatteryConsumerPlugin::UpdatePowerLoad(double _powerload)
{
    if (!this->battery->SetPowerLoad(this->consumer_id_, _powerload))
    {
        gzerr << "Error setting the consumer power load" << std::endl;
    }
}
GZ_REGISTER_MODEL_PLUGIN(RosBatteryConsumerPlugin)
} // namespace gazebo