#include <ros/ros.h>
#include <ros/sensor_msgs/JointState.h>
#include <sensor_msgs/JointState.h>
#include <map>
#include <aidu_elevator/actions/locatebutton.h>
#include <aidu_elevator/Button.h>
#include <aidu_robotarm/robot_arm_positions.h>

using namespace aidu::elevator;

LocateButton::LocateButton(ros::NodeHandle* nh) : Action::Action(nh) {
    
    // Set up subscribers
    buttonSubscriber = nh->subscribe<aidu_elevator::Button>("/elevator/button/classified", 1, &LocateButton::visibleButton, this);
    robotArmSubscriber = nh->subscribe<sensor_msgs::State>("/arm_state", 1, &LocateButton::updateArmState, this);
    
    // Set up publishers
    robotArmPublisher = nh->advertise<aidu_robotarm::robot_arm_positions>("/robot_arm_positions",1);
    
    // Initialize variables
    this->buttonFound = false;
    
    this->translationEpsilon = 0.0001;
    this->rotationEpsilon = 0.0005;
    
    this->translationStep = 0.05;
    this->rotationStep = 0.2;
    
    this->translationMaximum = 0.37;
    this->translationMinimum = 0.0;
    
    this->rotationMaximum = 1.57;
    this->rotationMinimum = -1.57;
    
}

LocateButton::~LocateButton() {
    
}

void LocateButton::execute() {
    //ROS_INFO("Executing locate button action");
    
    // Check if we achieved our current goal and are still moving
    if (!this->buttonFound && translationSpeed < translationEpsilon && rotationSpeed < rotationEpsilon) {
        
        // Set new wanted translation and rotation
        if (wantedTranslation < translationMaximum - translationStep) {
            wantedTranslation += translationStep;
        }
        if (wantedTranslation > translationMaximum - translationStep) {
            wantedTranslation = translationMinimum;
            wantedRotation += rotationStep;
        }
        if (wantedRotation > rotationMaximum - rotationStep) {
            wantedRotation = rotationMinimum;
        }
        
        // Send new position to arm
        aidu_robotarm::robot_arm_positions arm_position;
        arm_position.translation = 0.2;
        arm_position.rotation = 0.0;
        arm_position.extention = 0.0;
        robotArmPublisher.publish(arm_position);
    }
}

bool LocateButton::finished() {
    return this->buttonFound;
}

void LocateButton::visibleButton(const aidu_elevator::Button::ConstPtr& message) {
    if (message->button_type == aidu_elevator::Button::BUTTON_DOWN) {
        this->buttonFound = true;
    }
}

void LocateButton::updateArmState(const sensor_msgs::JointState::ConstPtr& message) {
    for(int i=0; i<message->name.size(); i++) {
        if (message->name[i] == "base_spindlecaddy") {
            translationSpeed = message->velocity[i];
            translation = message->position[i];
        } else if (message->name[i] == "spindlecaddy_rotationalarm") {
            rotationSpeed = message->velocity[i];
            rotation = message->position[i];
        }
    }
}







