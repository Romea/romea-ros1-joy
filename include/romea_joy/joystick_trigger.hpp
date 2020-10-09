#ifndef _JoystickTrigger_HPP
#define _JoystickTrigger_HPP

#include "joystick_axis.hpp"

namespace romea {

class JoystickTrigger : public JoystickAxis
{

public :

  JoystickTrigger(const int &axis_id);

  void update(const sensor_msgs::Joy & joy_msg)override;

private :

  double has_been_pressed_;

};

}

#endif
