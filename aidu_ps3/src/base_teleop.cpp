#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <aidu_ps3/base_teleop.h>
#include <geometry_msgs/Twist.h>
#include <math.h>


using namespace aidu;

ps3::TeleopBase::TeleopBase(): core::Node::Node() {

  
  //Subscribers and Publishers 
  basepublisher = nh->advertise<geometry_msgs::Twist>("/cmd_vel",1);
  joy_sub_ = nh->subscribe<sensor_msgs::Joy>("/joy", 1, &TeleopBase::joyCallback, this);
  ROS_INFO("teleop_base constructed");
  maxspeed=2.0*0.5*0.295; // m/s
  maxangle=0.4;	// rad/s 

}

void ps3::TeleopBase::joyCallback(const sensor_msgs::Joy::ConstPtr& joy){ 
  // publishing relevant joystick input on teleop topic
  geometry_msgs::Twist twist;
  float leftx,lefty ,r2,l2;
  //int circle=0,triangle=0,cross=0,square=0,l1=0,r1=0;
  leftx =joy->axes[0];
  lefty =joy->axes[1];
  r2=joy->axes[9];
  l2=joy->axes[8];
  
  //other buttons
  int up=joy->buttons[4];
  int down=joy->buttons[6];
  int left=joy->buttons[7];
  int right=joy->buttons[5];
  //circle=joy->buttons[13];
  //cross=joy->buttons[14];
  //triangle=joy->buttons[12];
  //square=joy->buttons[15];
  //r1=joy->buttons[11];
  //l1=joy->buttons[10];
  
  //adjustable maxspeed
  if (up==1 || down==1){
    maxspeed+=(up-down)*0.04;
  }
  //adjustable angular maxspeed
  if (left==1 || right==1){
    maxspeed+=(left-right)*0.05;
  }
  
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
