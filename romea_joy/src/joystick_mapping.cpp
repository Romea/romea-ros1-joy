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

#include "romea_joy/joystick_mapping.hpp"

namespace romea
{

//-----------------------------------------------------------------------------
JoystickMapping::JoystickMapping(const StringToStringMap &name_remappings,
                                 const bool & keep_only_remapped_ones):
  name_remappings_(name_remappings),
  keep_only_remapped_ones_(keep_only_remapped_ones)
{
}

//-----------------------------------------------------------------------------
JoystickMapping::StringToId JoystickMapping::get(const StringToId &id_mappings)const
{

  std::map<std::string,int> result;
  for(const auto & [name,id] : id_mappings)
  {
    if(auto it=name_remappings_.find(name);it!=name_remappings_.end())
    {
      result[it->second]=id;
    }
    else if(!keep_only_remapped_ones_)
    {
      result[name]=id;
    }
  }

  return result;
}

}

