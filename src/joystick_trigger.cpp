#include "romea_joy/joystick_trigger.hpp"

namespace romea
{

//-----------------------------------------------------------------------------
JoystickTrigger::JoystickTrigger(const int & axis_id):
  JoystickAxis(axis_id),
  has_been_pressed_(false)
{

}

//-----------------------------------------------------------------------------
void JoystickTrigger::update(const sensor_msgs::Joy & joy_msg)
{
  value_ =joy_msg.axes[id_];
  if(!has_been_pressed_)
  {
    if(std::abs(value_) >std::numeric_limits<double>::epsilon())
    {
      has_been_pressed_=true;
    }
    else
    {
      value_= 1;
    }
  }
}



}
