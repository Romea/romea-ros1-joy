#include "romea_joy/joystick_mapping.hpp"
#include <romea_common_utils/params/ros_param.hpp>

namespace romea
{

//-----------------------------------------------------------------------------
JoystickMapping::JoystickMapping(ros::NodeHandle & joy_nh):
  joy_nh_(joy_nh),
  name_remappings_(),
  keep_only_remapped_ones_(true)
{

}

//-----------------------------------------------------------------------------
JoystickMapping::JoystickMapping(ros::NodeHandle & joy_nh,
                                 const std::map<std::string,std::string> & name_remappings,
                                 const bool & keep_only_remapped_ones):
  joy_nh_(joy_nh),
  name_remappings_(name_remappings),
  keep_only_remapped_ones_(keep_only_remapped_ones)
{
}

//-----------------------------------------------------------------------------
std::map<std::string,int> JoystickMapping::get(const std::string & mappings_name)
{
  std::map<std::string,int> id_mappings = loadMap<int>(joy_nh_,mappings_name);
  if(name_remappings_.size()==0)
    return id_mappings;

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

