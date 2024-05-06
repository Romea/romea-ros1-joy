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

#ifndef _JoystickAxis_HPP
#define _JoystickAxis_HPP

#include <sensor_msgs/Joy.h>

namespace romea {

class JoystickAxis
{
public :

  using Ptr = std::unique_ptr<JoystickAxis>;

public :

  JoystickAxis(const int & axis_id);

  virtual ~JoystickAxis()=default;

  virtual void update(const sensor_msgs::Joy& joy_msg)=0;

  const double & getValue()const;

  const int & getId()const;


protected :

  int id_;
  double value_;
};

}
#endif
