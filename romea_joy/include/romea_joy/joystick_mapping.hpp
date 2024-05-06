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

#ifndef _romea_JoystickMapping_hpp_
#define _romea_JoystickMapping_hpp_

//ros
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

//std
#include <map>


namespace romea
{


class JoystickMapping
{

public :

  using StringToStringMap = std::map<std::string,std::string>;
  using StringToId = std::map<std::string,int>;

public :

  JoystickMapping(const StringToStringMap & name_remappings,
                  const bool & keep_only_remapped_ones);

  StringToId get(const StringToId &id_mappings) const;

private :

  StringToStringMap name_remappings_;
  bool keep_only_remapped_ones_;
};
}

#endif
