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

#ifndef _JoystickButton_HPP
#define _JoystickButton_HPP

//ros
#include <sensor_msgs/Joy.h>

//std
#include <functional>

namespace romea
{

class JoystickButton
{

public :

  using CallbackFunction = std::function<void(void)> ;

  using Ptr = std::unique_ptr<JoystickButton>;

  enum Event
  {
    PRESSED,
    RELEASED,
    TOGGLED
  };

public :

  JoystickButton(const int & button_id);

  void update(const sensor_msgs::Joy & joy_msg);

  void registerCallback(Event event, CallbackFunction &&function);

  const int & getValue()const;

  const int & getId()const;

private :

  int id_;
  int value_;

  CallbackFunction pressed_callback_;
  CallbackFunction released_callback_;
  CallbackFunction toggled_callback_;

};

}

#endif
