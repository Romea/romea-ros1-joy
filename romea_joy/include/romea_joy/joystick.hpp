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

#ifndef _Joystick_HPP_
#define _Joystick_HPP_

//ros
#include <ros/ros.h>

//romea
#include "joystick_button.hpp"
#include "joystick_stick.hpp"
#include "joystick_trigger.hpp"
#include "joystick_directional_pad.hpp"
#include "joystick_mapping.hpp"


//std
#include <functional>

namespace  romea
{


class Joystick
{
public :

  using OnReceivedMsgCallback = std::function<void(const Joystick &)>;

public :

  Joystick(ros::NodeHandle & nh,
           ros::NodeHandle & joy_nh);

  Joystick(ros::NodeHandle & nh,
           ros::NodeHandle & joy_nh,
           const std::map<std::string,std::string> & name_remappings,
           bool use_only_remapped);

  void registerOnReceivedMsgCallback(OnReceivedMsgCallback && callback);

  void registerButtonCallback(const std::string & button_name,
                              const JoystickButton::Event & event_type,
                              JoystickButton::CallbackFunction && callback);

  const int & getButtonValue(const std::string & button_name)const;

  const double & getAxisValue(const std::string & axis_name)const;

  const JoystickStick::Config & getSticksConfiguration()const;

  const JoystickTrigger::Config & getTriggersConfiguration()const;

private :

  void addButtons_(ros::NodeHandle & joy_nh, const JoystickMapping & joystick_mapping);
  void addDirectionalPads_(ros::NodeHandle &joy_nh, const JoystickMapping & joystick_mapping);
  void addSticks_(ros::NodeHandle &joy_nh, const JoystickMapping & joystick_mapping);
  void addTriggers_(ros::NodeHandle &joy_nh, const JoystickMapping & joystick_mapping);

  void processJoyMsg_(const sensor_msgs::Joy::ConstPtr & msg);

private :

  ros::Subscriber joy_sub_;
  std::map<std::string,JoystickButton::Ptr> buttons_;
  std::map<std::string,JoystickAxis::Ptr> axes_;
  JoystickStick::Config sticks_configuration_;
  JoystickTrigger::Config triggers_configuration_;
  OnReceivedMsgCallback on_received_msg_callback_;
};
}

#endif
