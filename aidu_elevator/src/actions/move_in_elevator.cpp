#include <ros/ros.h>
#include <aidu_elevator/actions/move_in_elevator.h>
#include <aidu_robotarm/robot_arm_positions.h>

using namespace aidu::elevator;

MoveInElevator::MoveInElevator(ros::NodeHandle* nh) : Action::Action(nh) {
    
    // Set up subscribers

    // Set up publishers
    basepositionPublisher= nh->advertise<geometry_msgs::Twist>("/pos",1);
    
    // Initialize variables

    
}

MoveInElevator::~MoveInElevator() {
    
}

void MoveInElevator::execute() {
      
}

bool MoveInElevator::finished() {

}








