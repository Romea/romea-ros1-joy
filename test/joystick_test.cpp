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
    stop(false),
    start_button_value(-1),
    stop_button_value(-1)

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

    joy->registerOnReceivedMsgCallback(std::bind(&TestJoystick::joystick_callback,this,std::placeholders::_1));

    msg.axes.resize(8);
    msg.buttons.resize(11);
    pub= nh.advertise<sensor_msgs::Joy>("joy",0);
  }

  void publish_joy_msg_and_wait()
  {
    pub.publish(msg);
    ros::spinOnce();
    ros::Duration(0.1).sleep();
    ros::spinOnce();
  }

  void joystick_callback(const romea::Joystick & joy)
  {
    start_button_value=joy.getButtonValue("start");
    stop_button_value=joy.getButtonValue("stop");
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
  int start_button_value;
  int stop_button_value;

};

//-----------------------------------------------------------------------------
TEST_F(TestJoystick, testPressedStartButton)
{
  msg.buttons[0]=0;
  msg.buttons[1]=0;
  publish_joy_msg_and_wait();
  EXPECT_EQ(start_button_value,0);
  EXPECT_EQ(stop_button_value,0);
  EXPECT_FALSE(start);
  EXPECT_FALSE(stop);

  msg.buttons[0]=1;
  publish_joy_msg_and_wait();
  EXPECT_EQ(start_button_value,1);
  EXPECT_EQ(stop_button_value,0);
  EXPECT_TRUE(start);
  EXPECT_FALSE(stop);
}

//-----------------------------------------------------------------------------
TEST_F(TestJoystick, testReleasedStopButton)
{
  ROS_ERROR_STREAM("testReleasedStopButton");

  msg.buttons[0]=0;
  msg.buttons[1]=1;
  publish_joy_msg_and_wait();
  EXPECT_EQ(start_button_value,0);
  EXPECT_EQ(stop_button_value,1);
  EXPECT_FALSE(start);
  EXPECT_FALSE(stop);

  ROS_ERROR_STREAM("testReleasedStopButton");

  msg.buttons[1]=0;
  publish_joy_msg_and_wait();
  EXPECT_EQ(start_button_value,0);
  EXPECT_EQ(stop_button_value,0);
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
