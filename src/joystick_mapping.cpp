#include "romea_joy/joystick_mapping.hpp"
#include <romea_common_utils/params/ros_param.hpp>

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
  for(const auto & id_map : id_mappings)
  {
    auto it=name_remappings_.find(id_map.first);
    if(it!=name_remappings_.end())
    {
      result[it->second]=id_map.second;
    }
    else if(!keep_only_remapped_ones_)
    {
      result[id_map.first]=id_map.second;
    }
  }

  return result;
}

}

