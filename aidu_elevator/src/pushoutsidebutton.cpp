#include <ros/ros.h>
#include <aidu_core/node.h>
#include <aidu_elevator/pushoutsidebutton.h>
#include <aidu_elevator/OutsideButton.h>

using namespace aidu;

OutsideButton::OutsideButton(): core::Node::Node() {
  //Subscribers and Publishers 
  pushoutsidebuttonSubsciber =  nh->subscribe<aidu_elevator::OutsideButton>("/outside", 1, &OutsideButton::OutsideButtonCallback, this);
}



void OutsideButton::OutsidebuttonCallback(const aidu_elevator::ElevatorNav::ConstPtr& outside){
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "elevator");
  aidu::Elevator elevator;
  elevator.spin();
  return 0;
}