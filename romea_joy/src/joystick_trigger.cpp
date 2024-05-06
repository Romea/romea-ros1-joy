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

#include "romea_joy/joystick_trigger.hpp"
namespace romea
{

//-----------------------------------------------------------------------------
JoystickTrigger::JoystickTrigger(const int & axis_id,
                                 const double & unpressed_value):
  JoystickAxis(axis_id),
  has_been_pressed_(false),
  unpressed_value_(unpressed_value)
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
      value_= unpressed_value_;
    }
  }
}



}
