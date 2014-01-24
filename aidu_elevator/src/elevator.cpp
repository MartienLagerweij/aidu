#include <ros/ros.h>
#include <aidu_core/node.h>
#include <aidu_elevator/elevator.h>
#include <aidu_elevator/actions/locatebutton.h>
#include <aidu_elevator/actions/pushbutton.h>
#include <aidu_elevator/OutsideButton.h>
#include <aidu_elevator/ElevatorNavigation.h>
#include <aidu_elevator/actions/go_to_door.h>
#include <aidu_elevator/Button.h>

using namespace aidu;

Elevator::Elevator(): core::Node::Node() {
    subscriber = nh->subscribe<aidu_elevator::ElevatorNavigation>("/elevator/navigation", 1, &Elevator::activate, this);
}

void Elevator::activate(const aidu_elevator::ElevatorNavigation::ConstPtr& message) {
    targetFloor = message->target_floor;
    currentFloor = message->current_floor;
    removeActions();
    setupActions();
}

void Elevator::removeActions() {
    while (this->currentAction != 0) {
        aidu::elevator::Action* next = this->currentAction->getNextAction();
        delete this->currentAction;
        this->currentAction = next;
    }
}

void Elevator::setupActions() {
    
    // Construct actions
    int outsideButton;
    if (currentFloor > targetFloor) {
        outsideButton = aidu_elevator::Button::BUTTON_DOWN;
    } else {
        outsideButton = aidu_elevator::Button::BUTTON_UP;
    }
    
    
    elevator::LocateButton* locateButton = new elevator::LocateButton(this->nh, outsideButton);
    elevator::PushButton* pushButton = new elevator::PushButton(this->nh, outsideButton);
    elevator::GoToDoor* goToDoor = new elevator::GoToDoor(this->nh);
    
    // Chain actions together
    locateButton->setNextAction(pushButton);
    pushButton->setNextAction(goToDoor);
    
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