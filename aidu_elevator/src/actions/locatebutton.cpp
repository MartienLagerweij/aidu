#include <ros/ros.h>
#include <aidu_elevator/actions/locatebutton.h>
#include <aidu_elevator/Button.h>

using namespace aidu::elevator;

LocateButton::LocateButton(ros::NodeHandle* nh) : Action::Action(nh) {
    buttonSubscriber = nh->subscribe<aidu_elevator::Button>("/elevator/button/classified", 1, &LocateButton::visibleButton, this);
    this->buttonFound = false;
}

LocateButton::~LocateButton() {
    
}

void LocateButton::execute() {
    ROS_INFO("Executing locate button action");
}

bool LocateButton::finished() {
    return this->buttonFound;
}

void LocateButton::visibleButton(const aidu_elevator::Button::ConstPtr& message) {
    if(message->button_type == aidu_elevator::Button::BUTTON_DOWN) {
        this->buttonFound = true;
    }
}