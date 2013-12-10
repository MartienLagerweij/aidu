#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <aidu_ps3/base_teleop.h>
#include <geometry_msgs/Twist.h>
#include <math.h>


using namespace aidu;

ps3::TeleopBase::TeleopBase(): core::Node::Node() {

  
  //Subscribers and Publishers 
  basepublisher = nh->advertise<geometry_msgs::Twist>("/base/speed",1);
  joy_sub_ = nh->subscribe<sensor_msgs::Joy>("/joy", 1, &TeleopBase::joyCallback, this);
  ROS_INFO("teleop_base constructed");
  maxspeed=0.5*0.5*0.295; // m/s
  maxangle=-0.06;	// rad/s 

}

void ps3::TeleopBase::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{ // publishing relevant joystick input on teleop topic
  geometry_msgs::Twist twist;
  float leftx,lefty ,r2,l2;
  //int circle=0,triangle=0,cross=0,square=0,l1=0,r1=0;
  leftx =joy->axes[0];
  lefty =joy->axes[1];
  r2=joy->axes[9];
  l2=joy->axes[8];
  
  //other buttons
  //circle=joy->buttons[13];
  //cross=joy->buttons[14];
  //triangle=joy->buttons[12];
  //square=joy->buttons[15];
  //r1=joy->buttons[11];
  //l1=joy->buttons[10];
  
  
  //exponential speeds with joystick and bottons
  twist.linear.x=(pow(r2,2)-pow(l2,2))*maxspeed;
  float sign=leftx<0 ? -1 : 1;
  twist.angular.z=sign*pow(leftx,2)*maxangle;
  
  ROS_INFO("PS3 Linear: %f", twist.linear.x);
  ROS_INFO("PS3 Angular: %f", twist.angular.z);
  
  
  /*
 //test speeds for plotting
 twist.linear.x=square*1+cross*4+triangle*6+circle*8;
 twist.angular.x=l1*1+l2*2-r1*1-r2*2;
 */
 
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
