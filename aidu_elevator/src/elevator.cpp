#include <ros/ros.h>
#include <aidu_core/node.h>
#include <aidu_elevator/elevator.h>
#include <aidu_elevator/actions/locatebutton.h>
#include <aidu_elevator/actions/pushbutton.h>
#include <aidu_elevator/OutsideButton.h>
#include <aidu_elevator/ElevatorNav.h>
#include <aidu_elevator/actions/go_to_door.h>

using namespace aidu;

Elevator::Elevator(): core::Node::Node() {
    setupActions();
}

void Elevator::setupActions() {
    
    // Construct actions
    elevator::LocateButton* locateButton = new elevator::LocateButton(this->nh);
    elevator::PushButton* PushButton = new elevator::PushButton(this->nh);
    elevator::GoToDoor* GoToDoor = new elevator::GoToDoor(this->nh);
    //...
    
    // Chain actions together
    //locateButton.setNextAction(PushButton);
    //...
    
    // Set the first action as current action
    this->currentAction = GoToDoor;
    
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
        ros::spinOnce();
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