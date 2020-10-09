//gtest
#include <gtest/gtest.h>

//ros
#include <ros/ros.h>

//romea
#include "romea_joy/joystick_mapping.hpp"

TEST(TestJoyMapping, testXboxMapping)
{
  ros::NodeHandle private_nh("~");
  romea::JoystickMapping mapping(private_nh);

  auto buttons_mappings=mapping.get("buttons");
  EXPECT_EQ(buttons_mappings.at("A"),0);
  EXPECT_EQ(buttons_mappings.at("B"),1);
  EXPECT_EQ(buttons_mappings.at("X"),2);
  EXPECT_EQ(buttons_mappings.at("Y"),3);
  EXPECT_EQ(buttons_mappings.at("LB"),4);
  EXPECT_EQ(buttons_mappings.at("RB"),5);
  EXPECT_EQ(buttons_mappings.at("back"),6);
  EXPECT_EQ(buttons_mappings.at("start"),7);
  EXPECT_EQ(buttons_mappings.at("power"),8);
  EXPECT_EQ(buttons_mappings.at("LSB"),9);
  EXPECT_EQ(buttons_mappings.at("RSB"),10);

  auto sticks_mappings=mapping.get("axes/sticks");
  EXPECT_EQ(sticks_mappings.at("HLS"),0);
  EXPECT_EQ(sticks_mappings.at("VLS"),1);
  EXPECT_EQ(sticks_mappings.at("HRS"),2);
  EXPECT_EQ(sticks_mappings.at("VRS"),3);

  auto triggers_mappings=mapping.get("axes/triggers");
  EXPECT_EQ(triggers_mappings.at("RT"),4);
  EXPECT_EQ(triggers_mappings.at("LT"),5);

  auto dpad_mappings=mapping.get("axes/directional_pads");
  EXPECT_EQ(dpad_mappings.at("HDPAD"),6);
  EXPECT_EQ(dpad_mappings.at("VDPAD"),7);
}



int main(int argc, char** argv)
{

  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "joy_mapping");

  int ret = RUN_ALL_TESTS();
  ros::shutdown();
  return ret;
}
