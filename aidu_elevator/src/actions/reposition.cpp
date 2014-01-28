#include <ros/ros.h>
#include <aidu_elevator/actions/reposition.h>
#include <geometry_msgs/Twist.h>

using namespace aidu::elevator;

Reposition::Reposition(ros::NodeHandle* nh) : Action::Action(nh) {
    // Position publisher
    positionPublisher = nh->advertise<geometry_msgs::Twist>("/pos",1,true);
    done = false;
}

Reposition::~Reposition() {
    
}

void Reposition::execute() {
  ROS_INFO("Executing reposition action");
  sleep(1);
  this->moveBase(0.0, 1.57/2.0);
  sleep(2);
  this->moveBase(-0.2, 0.0);
  sleep(1);
  this->moveBase(0.0, -1.57/2.0);
  sleep(2);
  ROS_INFO("Reposition complete");
  this->done = true;
}

void Reposition::moveBase(double linear, double angular) {
    geometry_msgs::Twist position;
    position.angular.z = angular;
    position.linear.x = linear;
    positionPublisher.publish(position);
    ros::spinOnce();
}

bool Reposition::finished() {
  return this->done;
}
