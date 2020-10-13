// gtest
#include <gtest/gtest.h>

#include "romea_joy/joystick.hpp"



class TestJoystick : public ::testing::Test
{
public :

  TestJoystick():
    joy(nullptr),
    msg(),
    pub(),
    start(false),
    stop(false)
  {

  }


  void SetUp()
  {
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    std::map<std::string,std::string> remappings;
    remappings["A"]="start";
    remappings["B"]="stop";
    joy=std::make_unique<romea::Joystick>(nh,private_nh,remappings,true);

    joy->registerButtonCallback("start",
                                     romea::JoystickButton::PRESSED,
                                     std::bind(&TestJoystick::start_callback,this));

    joy->registerButtonCallback("stop",
                                     romea::JoystickButton::RELEASED,
                                     std::bind(&TestJoystick::stop_callback,this));
    msg.axes.resize(8);
    msg.buttons.resize(11);
    pub= nh.advertise<sensor_msgs::Joy>("joy",0);
  }


  void start_callback()
  {
    start=true;
  }

  void stop_callback()
  {
    stop=true;
  }

  std::unique_ptr<romea::Joystick> joy;
  sensor_msgs::Joy msg;
  ros::Publisher pub;
  bool start;
  bool stop;

};

//-----------------------------------------------------------------------------
TEST_F(TestJoystick, testPressedStartButton)
{
  msg.buttons[0]=0;
  pub.publish(msg);
  ros::spinOnce();
  EXPECT_FALSE(start);
  EXPECT_FALSE(stop);

  msg.buttons[0]=1;
  pub.publish(msg);
  ros::spinOnce();
  EXPECT_TRUE(start);
  EXPECT_FALSE(stop);
}

//-----------------------------------------------------------------------------
TEST_F(TestJoystick, testReleasedStopButton)
{
  msg.buttons[1]=1;
  pub.publish(msg);
  ros::spinOnce();
  EXPECT_FALSE(start);
  EXPECT_FALSE(stop);

  msg.buttons[1]=0;
  pub.publish(msg);
  ros::spinOnce();
  EXPECT_FALSE(start);
  EXPECT_TRUE(stop);
}


//-----------------------------------------------------------------------------
int main(int argc, char **argv){

  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "joy_mapping");

  int ret = RUN_ALL_TESTS();
  ros::shutdown();
  return ret;
}
