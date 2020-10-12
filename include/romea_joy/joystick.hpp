#ifndef _Joystick_HPP_
#define _Joystick_HPP_

//ros
#include <ros/ros.h>

//romea
#include "joystick_button.hpp"
#include "joystick_stick.hpp"
#include "joystick_trigger.hpp"

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

  void addButtons_(const std::map<std::string, int> &id_mappings);
  void addDirectionalPads_(const std::map<std::string,int> & id_mappings);
  void addSticks_(const std::map<std::string, int> &id_mappings, const double & deadzone);
  void addTriggers_(const std::map<std::string,int> & id_mappings, const double & unpressed_value);

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
