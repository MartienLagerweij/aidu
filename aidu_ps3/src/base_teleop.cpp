#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <aidu_ps3/base_teleop.h>
#include <geometry_msgs/Twist.h>

using namespace aidu;

ps3::TeleopBase::TeleopBase(): core::Node::Node() {

  
  //Subscribers and Publishers 
  basepublisher = nh->advertise<geometry_msgs::Twist>("/base/twist",1);
  joy_sub_ = nh->subscribe<sensor_msgs::Joy>("/joy", 1, &TeleopBase::joyCallback, this);
  ROS_INFO("teleop_base constructed");
  maxspeed=4;
  maxangle=-1;

}

void ps3::TeleopBase::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{ // publishing relevant joystick input on teleop topic
  geometry_msgs::Twist twist;
  float leftx,lefty,circle;
  leftx =joy->axes[0];
  lefty =joy->axes[1];
  circle=joy->buttons[13];
  if (circle!=1){
    twist.linear.x=maxspeed*lefty;
    twist.angular.x=maxangle*leftx;
  }else {
    twist.linear.x=0;
    twist.angular.x=0;
  }
  basepublisher.publish(twist);
  ros::spinOnce();
  
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_base");;
  ps3::TeleopBase teleop_base;
  teleop_base.spin();
  return 0;
}
