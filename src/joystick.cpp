#include "romea_joy/joystick.hpp"
#include "romea_joy/joystick_stick.hpp"
#include "romea_joy/joystick_trigger.hpp"
#include "romea_joy/joystick_directional_pad.hpp"
#include "romea_joy/joystick_mapping.hpp"
#include <romea_common_utils/params/ros_param.hpp>

namespace romea
{
  JoystickStick::Config loadSticksConfiguration(ros::NodeHandle nh, const std::string paramName)
  {
    std::vector<double> interval =load_vector<double>(nh,paramName);
    if(interval.size()!=2)
    {
      throw(std::runtime_error("Unable load sticks configuration from ros parameter " + paramName));
    }
    return {interval[0],interval[1]};
  }

  JoystickTrigger::Config loadTriggersConfiguration(ros::NodeHandle nh, const std::string paramName)
  {
    std::vector<double> interval =load_vector<double>(nh,paramName);
    if(interval.size()!=2)
    {
      throw(std::runtime_error("Unable load triggers configuration from ros parameter " + paramName));
    }
    return {interval[0],interval[1]};
  }

}

namespace romea
{

//-----------------------------------------------------------------------------
Joystick::Joystick(ros::NodeHandle & nh,
                   ros::NodeHandle & joy_nh):
  Joystick(nh,
           joy_nh,
           std::map<std::string,std::string>(),
           false)
{

}

//-----------------------------------------------------------------------------
Joystick::Joystick(ros::NodeHandle & nh,
                   ros::NodeHandle & joy_nh,
                   const std::map<std::string,std::string> & name_remappings,
                   bool use_only_remapped):
  joy_sub_(),
  buttons_(),
  axes_(),
  sticks_configuration_(),
  triggers_configuration_()
{
  double deadzone = load_param<double>(joy_nh,"deadzone");
  sticks_configuration_=loadSticksConfiguration(joy_nh,"scale/sticks");
  triggers_configuration_=loadTriggersConfiguration(joy_nh,"scale/triggers");

  JoystickMapping joystick_mapping(joy_nh,name_remappings,use_only_remapped);
  addButtons_(joystick_mapping.get("buttons"));
  addDirectionalPads_(joystick_mapping.get("axes/directional_pads"));
  addSticks_(joystick_mapping.get("axes/sticks"),deadzone);
  addTriggers_(joystick_mapping.get("axes/triggers"),
               triggers_configuration_.unpressed_value);

  joy_sub_=nh.subscribe<sensor_msgs::Joy>("joy", 1, &Joystick::processJoyMsg_,this);
}

//-----------------------------------------------------------------------------
const JoystickStick::Config & Joystick::getSticksConfiguration()const
{
  return sticks_configuration_;
}

//-----------------------------------------------------------------------------
const JoystickTrigger::Config & Joystick::getTriggersConfiguration()const
{
  return triggers_configuration_;
}

//-----------------------------------------------------------------------------
void Joystick::processJoyMsg_(const sensor_msgs::Joy::ConstPtr & msg)
{

  if(msg->axes.empty() || msg->buttons.empty())
  {
    ROS_ERROR_STREAM("Unavailable joy msg : check if joy node is connected to a proper device");
  }
  else
  {

    for(auto & p :buttons_)
    {
      p.second->update(*msg);
    }

    for(auto & p : axes_)
    {
      p.second->update(*msg);
    }

    if(on_received_msg_callback_)
    {
      on_received_msg_callback_(*this);
    }
  }
}


//-----------------------------------------------------------------------------
void Joystick::registerButtonCallback(const std::string & button_name,
                                      const JoystickButton::Event & event_type,
                                      JoystickButton::CallbackFunction &&callback)
{
  auto it = buttons_.find(button_name);
  if(it == buttons_.end())
  {
    std::stringstream ss;
    ss << "No joystick button called ";
    ss << button_name;
    ss << " has been found, register ";
    ss << button_name;
    ss << " callback failed. Check joystick buttons mappings or remappings ";
    throw(std::runtime_error(ss.str()));
  }

  it->second->registerCallback(event_type,std::move(callback));
}

//-----------------------------------------------------------------------------
void Joystick::registerOnReceivedMsgCallback(OnReceivedMsgCallback && callback)
{
  on_received_msg_callback_=callback;
}


//-----------------------------------------------------------------------------
void Joystick::addButtons_(const std::map<std::string,int> & id_mappings)
{
  for(const auto & p : id_mappings)
  {
    buttons_[p.first]=std::make_unique<JoystickButton>(p.second);
  }
}

//-----------------------------------------------------------------------------
void Joystick::addDirectionalPads_(const std::map<std::string, int> &id_mappings)
{
  for(const auto & p : id_mappings)
  {
    axes_[p.first]=std::make_unique<JoystickDirectionalPad>(p.second);
  }
}
//-----------------------------------------------------------------------------
void Joystick::addSticks_(const std::map<std::string,int> & id_mappings,
                          const double & deadzone)
{
  for(const auto & p : id_mappings)
  {
    axes_[p.first]=std::make_unique<JoystickStick>(p.second,deadzone);
  }

}
//-----------------------------------------------------------------------------
void Joystick::addTriggers_(const std::map<std::string, int> &id_mappings,
                            const double & unpressed_value)
{
  for(const auto & p : id_mappings)
  {
    axes_[p.first]=std::make_unique<JoystickTrigger>(p.second,unpressed_value);
  }
}

//-----------------------------------------------------------------------------
const int & Joystick::getButtonValue(const std::string & button_name)const
{
  return buttons_.at(button_name)->getValue();
}

//-----------------------------------------------------------------------------
const double & Joystick::getAxisValue(const std::string & axis_name)const
{
  return axes_.at(axis_name)->getValue();
}


}
