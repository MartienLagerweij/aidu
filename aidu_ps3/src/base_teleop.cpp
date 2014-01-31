#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <aidu_ps3/base_teleop.h>
#include <aidu_elevator/ElevatorNavigation.h>
#include <geometry_msgs/Twist.h>
#include <aidu_robotarm/robot_arm_positions.h>
#include <math.h>


using namespace aidu;

ps3::TeleopBase::TeleopBase(): core::Node::Node() {

  
  //Subscribers and Publishers 
  basepublisher = nh->advertise<geometry_msgs::Twist>("/cmd_vel",1);
  elevatorpublisher= nh ->advertise<aidu_elevator::ElevatorNavigation>("/elevator/navigation",1);
  robotarmpublisher = nh->advertise<aidu_robotarm::robot_arm_positions>("/robot_arm_positions", 1);
  robotarmspeedpub = nh->advertise<geometry_msgs::Twist>("/robot_arm_speed", 1);
  joy_sub_ = nh->subscribe<sensor_msgs::Joy>("/joy", 1, &TeleopBase::joyCallback, this);
  ROS_INFO("teleop_base constructed");
  maxspeed=2.0*0.5*0.295; // m/s
  maxangle=0.4;	// rad/s 
  active = true;

}

void ps3::TeleopBase::joyCallback(const sensor_msgs::Joy::ConstPtr& joy){ 
  // publishing relevant joystick input on teleop topic
  geometry_msgs::Twist twist;
  float leftx,lefty,rightx,righty,r2,l2;
  //int circle=0,triangle=0,cross=0,square=0,l1=0,r1=0;
  leftx =joy->axes[0];
  lefty =joy->axes[1];
  rightx = joy->axes[2];
  righty = joy->axes[3];
  r2=joy->axes[9];
  l2=joy->axes[8];
  
  //other buttons
  int up=joy->buttons[4];
  int down=joy->buttons[6];
  int left=joy->buttons[7];
  int right=joy->buttons[5];
  int circle=joy->buttons[13];
  int cross=joy->buttons[14];
  int triangle=joy->buttons[12];
  int square=joy->buttons[15];
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
  
  //ROS_INFO("PS3 Linear: %f", twist.linear.x);
  //ROS_INFO("PS3 Angular: %f", twist.angular.z);
  
  aidu_elevator::ElevatorNavigation elevator;
  if (triangle==1){
    elevator.current_floor=0;
    elevator.target_floor=2;
    elevatorpublisher.publish(elevator);
    
    active = false;
  }
  if (square==1) {
    active = true;
  }
  
  if(active) {
  geometry_msgs::Twist twistspeed;
  twistspeed.linear.x = 2*(cross - circle);
  twistspeed.linear.y = rightx / 2.0;
  twistspeed.linear.z = righty * 20;
  robotarmspeedpub.publish(twistspeed);
  
  basepublisher.publish(twist);
  }
    
  
  
  /*
 //test speeds for plotting
 twist.linear.x=square*1+cross*4+triangle*6+circle*8;
 twist.angular.x=l1*1+l2*2-r1*1-r2*2;
 */
 
  ros::spinOnce();
  
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_base");;
  ps3::TeleopBase teleop_base;
  teleop_base.spin();
  return 0;
}
