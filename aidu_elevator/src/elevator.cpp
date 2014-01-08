#include <ros/ros.h>
#include <aidu_core/node.h>
#include <aidu_elevator/elevator.h>
#include <aidu_elevator/OutsideButton.h>
#include <aidu_elevator/ElevatorNav.h>

using namespace aidu;

Elevator::Elevator(): core::Node::Node() {
  //Subscribers and Publishers 
  pushoutsidebuttonPubliher = nh->advertise<aidu_elevator::OutsideButton>("/outside",1);
  elevatorSubscriber= nh->subscribe<aidu_elevator::ElevatorNav>("/elevator_initiate", 1, &Elevator::initiateCallback, this);
}



void Elevator::initiateCallback(const aidu_elevator::ElevatorNav::ConstPtr& elevator_initiate){
  aidu_elevator::OutsideButton outside;
  if (elevator_initiate->current_floor > elevator_initiate->target_floor){
    outside.up=false;
  }
  else if(elevator_initiate->current_floor < elevator_initiate->target_floor){
    outside.up=true;
  }
  if (elevator_initiate->current_floor == elevator_initiate->target_floor){
    ROS_INFO("already at correct floor");
  }
  else {
    pushoutsidebuttonPubliher.publish(outside);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "elevator");
  aidu::Elevator elevator;
  elevator.spin();
  return 0;
}