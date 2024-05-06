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

// gtest
#include <gtest/gtest.h>

#include "romea_joy/joystick_stick.hpp"

class TestStick : public ::testing::Test
{
public :

  TestStick():
    stick1(0,0.05),
    stick2(1,0.05),
    msg()
  {

  }

  void SetUp()
  {
    msg.axes.resize(8);
    msg.buttons.resize(11);
  }

  romea::JoystickStick stick1;
  romea::JoystickStick stick2;
  sensor_msgs::Joy msg;
};

//-----------------------------------------------------------------------------
TEST_F(TestStick, testInsideDeadZone)
{
  msg.axes[0]=0.05;
  msg.axes[1]=-0.05;

  stick1.update(msg);
  EXPECT_NEAR(stick1.getValue(),0.,0.001);

  stick2.update(msg);
  EXPECT_NEAR(stick1.getValue(),0.,0.001);

}

//-----------------------------------------------------------------------------
TEST_F(TestStick, testMax)
{
  msg.axes[0]=1;
  msg.axes[1]=-1;

  stick1.update(msg);
  EXPECT_DOUBLE_EQ(stick1.getValue(),1.);

  stick2.update(msg);
  EXPECT_DOUBLE_EQ(stick1.getValue(),1.);
}

//-----------------------------------------------------------------------------
int main(int argc, char **argv){
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
