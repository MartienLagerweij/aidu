#include <ros/ros.h>
#include <aidu_elevator/actions/locatebutton.h>
#include <aidu_elevator/Button.h>
#include <aidu_robotarm/robot_arm_positions.h>

using namespace aidu::elevator;

LocateButton::LocateButton(ros::NodeHandle* nh) : Action::Action(nh) {
    //subscriber
    buttonSubscriber = nh->subscribe<aidu_elevator::Button>("/elevator/button/classified", 1, &LocateButton::visibleButton, this);
    //publisher
    robot_arm_positions_pub= nh->advertise<aidu_robotarm::robot_arm_positions>("/robot_arm_positions",1);
    this->buttonFound = false;
    
}

LocateButton::~LocateButton() {
    
}

void LocateButton::execute() {
      //ROS_INFO("Executing locate button action");
      aidu_robotarm::robot_arm_positions arm_position;
      arm_position.translation=0.2;
      arm_position.rotation=1.57;
      arm_position.extention=0.0;
      robot_arm_positions_pub.publish(arm_position);
}

bool LocateButton::finished() {
    return this->buttonFound;
}

void LocateButton::visibleButton(const aidu_elevator::Button::ConstPtr& message) {
    if(message->button_type == aidu_elevator::Button::BUTTON_DOWN) {
        this->buttonFound = true;
    }
}