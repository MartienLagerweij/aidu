#include <ros/ros.h>
#include <aidu_core/node.h>
#include <aidu_elevator/elevator.h>
#include <aidu_elevator/actions/locatebutton.h>
#include <aidu_elevator/actions/pushbutton.h>
#include <aidu_elevator/OutsideButton.h>
#include <aidu_elevator/ElevatorNavigation.h>
#include <aidu_elevator/actions/go_to_door.h>
#include <aidu_elevator/actions/movetobutton.h>
#include <aidu_elevator/actions/move_in_elevator.h>
#include <aidu_elevator/Button.h>

using namespace aidu;

Elevator::Elevator(): core::Node::Node() {
    subscriber = nh->subscribe<aidu_elevator::ElevatorNavigation>("/elevator/navigation", 1, &Elevator::activate, this);
    this->currentAction = 0;
}

void Elevator::activate(const aidu_elevator::ElevatorNavigation::ConstPtr& message) {
    targetFloor = message->target_floor;
    currentFloor = message->current_floor;
    //removeActions();
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
    
    int insideButton;
    switch(targetFloor) {
      case -1: insideButton = aidu_elevator::Button::BUTTON_K; break;
      case 0: insideButton = aidu_elevator::Button::BUTTON_B; break;
      case 1: insideButton = aidu_elevator::Button::BUTTON_1; break;
      case 2: insideButton = aidu_elevator::Button::BUTTON_2; break;
      case 3: insideButton = aidu_elevator::Button::BUTTON_3; break;
      case 4: insideButton = aidu_elevator::Button::BUTTON_4; break;
      default: ROS_ERROR("Unknown target floor: %d", targetFloor); return;
    }
    
    elevator::LocateButton* locateButtonOutside = new elevator::LocateButton(this->nh, outsideButton, 0.0);
    elevator::MoveToButton* moveToButtonOutside = new elevator::MoveToButton(this->nh, outsideButton);
    elevator::PushButton* pushButtonOutside = new elevator::PushButton(this->nh, outsideButton);
    elevator::GoToDoor* goToDoor = new elevator::GoToDoor(this->nh);
    elevator::MoveInElevator* moveInElevator = new elevator::MoveInElevator(this->nh);
    elevator::LocateButton* locateButtonInside = new elevator::LocateButton(this->nh, insideButton, -1.56);
    elevator::PushButton* pushButtonInside = new elevator::PushButton(this->nh, insideButton);
    
    // Chain actions together
    locateButtonOutside->setNextAction(moveToButtonOutside);
    moveToButtonOutside->setNextAction(pushButtonOutside);
    pushButtonOutside->setNextAction(goToDoor);
    goToDoor->setNextAction(moveInElevator);
    moveInElevator->setNextAction(locateButtonInside);
    locateButtonInside->setNextAction(pushButtonInside);
    
    // Set the first action as current action
    this->currentAction = moveInElevator;
    
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