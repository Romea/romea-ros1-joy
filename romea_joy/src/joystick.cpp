// Copyright 2024 INRAE, French National Research Institute for Agriculture, Food and Environment
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "romea_joy/joystick.hpp"
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

  JoystickMapping joystick_mapping(name_remappings,use_only_remapped);

  addButtons_(joy_nh,joystick_mapping);
  addDirectionalPads_(joy_nh,joystick_mapping);
  addSticks_(joy_nh,joystick_mapping);
  addTriggers_(joy_nh,joystick_mapping);

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

  if(auto it = buttons_.find(button_name);it == buttons_.end())
  {
    std::stringstream ss;
    ss << "No joystick button called ";
    ss << button_name;
    ss << " has been found, register ";
    ss << button_name;
    ss << " callback failed. Check joystick buttons mappings or remappings ";
    throw(std::runtime_error(ss.str()));
  }
  else
  {
    it->second->registerCallback(event_type,std::move(callback));
  }

}

//-----------------------------------------------------------------------------
void Joystick::registerOnReceivedMsgCallback(OnReceivedMsgCallback && callback)
{
  on_received_msg_callback_=callback;
}


//-----------------------------------------------------------------------------
void Joystick::addButtons_(ros::NodeHandle & joy_nh,
                           const JoystickMapping & joystick_mapping)
{
  auto mapping = load_map<int>(joy_nh,"buttons/mapping");

  for(const auto & [buttom_name, button_id] : joystick_mapping.get(mapping))
  {
    buttons_[buttom_name]=std::make_unique<JoystickButton>(button_id);
  }

}

//-----------------------------------------------------------------------------
void Joystick::addDirectionalPads_(ros::NodeHandle & joy_nh,
                                   const JoystickMapping & joystick_mapping)
{
  if(joy_nh.hasParam("axes/directional_pads"))
  {

    auto mapping = load_map<int>(joy_nh,"axes/directional_pads/mapping");

    for(const auto & [axe_name,axe_id] : joystick_mapping.get(mapping))
    {
      axes_[axe_name]=std::make_unique<JoystickDirectionalPad>(axe_id);
    }

  }
}
//-----------------------------------------------------------------------------
void Joystick::addSticks_(ros::NodeHandle & joy_nh,
                          const JoystickMapping & joystick_mapping)
{
  double deadzone = load_param<double>(joy_nh,"deadzone");

  sticks_configuration_=loadSticksConfiguration(joy_nh,"axes/sticks/scale");

  auto mapping = load_map<int>(joy_nh,"axes/sticks/mapping");

  for(const auto & [stick_name, stick_id] : joystick_mapping.get(mapping))
  {
    axes_[stick_name]=std::make_unique<JoystickStick>(stick_id,deadzone);
  }

}
//-----------------------------------------------------------------------------
void Joystick::addTriggers_(ros::NodeHandle & joy_nh,
                            const JoystickMapping & joystick_mapping)
{

  if(joy_nh.hasParam("axes/triggers"))
  {

    triggers_configuration_=loadTriggersConfiguration(joy_nh,"axes/triggers/scale");

    std::map<std::string,int> mapping = load_map<int>(joy_nh,"axes/triggers/mapping");

    for(const auto & [trigger_name,trigger_id] : joystick_mapping.get(mapping))
    {
      axes_[trigger_name]=std::make_unique<JoystickTrigger>(trigger_id,triggers_configuration_.unpressed_value);
    }
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
