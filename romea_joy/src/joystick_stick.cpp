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

#include "romea_joy/joystick_stick.hpp"

namespace romea
{

//-----------------------------------------------------------------------------
JoystickStick::JoystickStick(const int & axis_id,
                             const double & deadzone):
  JoystickAxis(axis_id),
  deadzone_(deadzone),
  scale_(1./(1-deadzone))
{

}

//-----------------------------------------------------------------------------
void JoystickStick::update(const sensor_msgs::Joy & joy_msg)
{
  double val = joy_msg.axes[id_];
  if(std::abs(val) < deadzone_)
  {
    value_= 0;
  }
  else
  {
    if(val>0)
    {
      value_=(val-deadzone_)*scale_;
    }
    else
    {
      value_=(val+deadzone_)*scale_;
    }
  }
}



}
