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

  JoystickMapping(ros::NodeHandle & joy_nh);

  JoystickMapping(ros::NodeHandle & joy_nh,
                  const std::map<std::string,std::string> & name_remappings,
                  const bool & keep_only_remapped_ones);

  std::map<std::string,int> get(const std::string & mappings_name);

private :

  ros::NodeHandle & joy_nh_;
  std::map<std::string,std::string> name_remappings_;
  bool keep_only_remapped_ones_;
};
}

#endif
