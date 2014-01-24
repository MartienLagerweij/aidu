#include <ros/ros.h>
#include <aidu_elevator/actions/go_to_door.h>
#include <geometry_msgs/Twist.h>
#include <aidu_vision/DistanceSensors.h>

using namespace aidu::elevator;

GoToDoor::GoToDoor(ros::NodeHandle* nh) : Action::Action(nh) {
    //publisher
    position_pub = nh->advertise<geometry_msgs::Twist>("/pos",1,true);
    //subscriber
    sensor_sub = nh->subscribe("/sensors", 1, &GoToDoor::sensorcallback, this);
    this->door_open = false;
    distance=0.0;
}

GoToDoor::~GoToDoor() {
    
}

void GoToDoor::execute() {
  bool door_open=false;
  ROS_INFO("Executing Go to door action");
  sleep(1);
  
  geometry_msgs::Twist position;
  position.angular.z=0.0;
  position.linear.x=-0.4;
  position_pub.publish(position);
  ros::spinOnce();
  sleep(2);
  position.angular.z=-1.57075;
  position.linear.x=0.0;
  position_pub.publish(position);
  ros::spinOnce();
  sleep(2);
  position.angular.z=0.0;
  position.linear.x=0.8;
  position_pub.publish(position);
  ros::spinOnce();
  sleep(3);
  position.angular.z=1.57075;
  position.linear.x=0.0;
  position_pub.publish(position);
  ros::spinOnce();
  sleep(2);
  position.angular.z=0.0;
  position.linear.x=0.2;
  position_pub.publish(position);
  ros::spinOnce();
  sleep(2);
  ros::Rate loopRate(10);
  while (distance<1.5 && ros::ok()){
        ros::spinOnce();
	loopRate.sleep();
  }
  this->door_open=true;
  ROS_INFO("door open");
}

bool GoToDoor::finished() {
  return this->door_open;
}

void GoToDoor::sensorcallback(const aidu_vision::DistanceSensors::ConstPtr& dist_msg){
  distance=(dist_msg->Frontleft+dist_msg->Frontright)/2.0;
  //ROS_INFO("distance:%f",distance);
}