#include <ros/ros.h>
#include <aidu_elevator/actions/pushbutton.h>
#include <aidu_elevator/Button.h>
#include <aidu_robotarm/robot_arm_positions.h>

using namespace aidu::elevator;

PushButton::PushButton(ros::NodeHandle* nh) : Action::Action(nh) {
    //subscriber
    buttonSubscriber = nh->subscribe<aidu_elevator::Button>("/elevator/button/classified", 1, &PushButton::visibleButton, this);
    //publisher
    robot_arm_positions_pub= nh->advertise<aidu_robotarm::robot_arm_positions>("/robot_arm_positions",1);
    this->buttonPushed = false;
    
}

PushButton::~PushButton() {
    
}

void PushButton::execute() {
      ROS_INFO("Executing push button action");
      aidu_robotarm::robot_arm_positions arm_position;
  
      robot_arm_positions_pub.publish(arm_position);
}

bool PushButton::finished() {
    return this->buttonPushed;
}

void PushButton::visibleButton(const aidu_elevator::Button::ConstPtr& message) {
}