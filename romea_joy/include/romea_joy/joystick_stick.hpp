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

#ifndef _JoystickStick_HPP
#define _JoystickStick_HPP

#include "joystick_axis.hpp"

namespace romea {

class JoystickStick : public JoystickAxis
{

public:

  struct Config
  {
    double left_or_down_value;
    double right_or_up_value;
  };

public :

  JoystickStick(const int & axis_id,
                const double & deadzone);

  virtual ~JoystickStick()=default;

  virtual void update(const sensor_msgs::Joy& joy_msg)override;

private :

  double deadzone_;
  double scale_;
};

}
#endif
