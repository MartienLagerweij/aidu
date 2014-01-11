#include <ros/ros.h>
#include <aidu_core/node.h>
#include <aidu_elevator/elevator.h>
#include <aidu_elevator/actions/locatebutton.h>
#include <aidu_elevator/OutsideButton.h>
#include <aidu_elevator/ElevatorNav.h>

using namespace aidu;

Elevator::Elevator(): core::Node::Node() {
    
}

void Elevator::setupActions() {
    
    // Construct actions
    elevator::LocateButton* locateButton = new elevator::LocateButton(this->nh);
    //elevator::MoveToFront* moveToFront = new elevator::MoveToFront(this->nh);
    //...
    
    // Chain actions together
    //locateButton.setNextAction(moveToFront);
    //...
    
    // Set the first action as current action
    this->currentAction = locateButton;
    
}

void Elevator::spin() {
    ros::Rate loopRate(20);
    while(ros::ok()) {
        
        if(this->currentAction != 0) {
            this->currentAction->execute();
            if(this->currentAction->finished()) {
                aidu::elevator::Action* next = this->currentAction->getNextAction();
                delete this->currentAction;
                this->currentAction = next;
            }
        }
        
        loopRate.sleep();
    }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "elevator");
  aidu::Elevator elevator;
  elevator.spin();
  return 0;
}