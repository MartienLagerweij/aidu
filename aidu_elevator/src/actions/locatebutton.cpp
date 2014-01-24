#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <map>
#include <aidu_elevator/actions/locatebutton.h>
#include <aidu_elevator/Button.h>
#include <aidu_robotarm/robot_arm_positions.h>

using namespace aidu::elevator;

LocateButton::LocateButton(ros::NodeHandle* nh) : Action::Action(nh) {
    
    // Set up subscribers
    buttonSubscriber = nh->subscribe<aidu_elevator::Button>("/elevator/button/classified", 1, &LocateButton::visibleButton, this);
    robotArmSubscriber = nh->subscribe<sensor_msgs::JointState>("/arm_state", 1, &LocateButton::updateArmState, this);
    
    // Set up publishers
    robotArmPublisher = nh->advertise<aidu_robotarm::robot_arm_positions>("/robot_arm_positions",1);
    
    // Initialize variables
    this->buttonFound = false;
    
    this->translationEpsilon = 0.0031;
    this->rotationEpsilon = 0.031;
    
    this->translationStep = 0.05;
    this->rotationStep = 0.785/2.0;
    
    this->translationMaximum = 0.37;
    this->translationMinimum = 0.0;
    
    this->rotationMaximum = 0.786/2.0;
    this->rotationMinimum = -0.786/2.0;
    
    this->translation = -0.1;
    this->rotation = 0.0;
    
    this->translationSpeed = 0.0;
    this->rotationSpeed = 0.0;
    
    this->wantedRotation = 0.0;
    this->wantedTranslation = translationMinimum;
    
}

LocateButton::~LocateButton() {
    
}

void LocateButton::execute() {
    //ROS_INFO("Executing locate button action");
  
    //ROS_INFO("Translation: v=%.5f d=%.5f - wanted=%.5f step=%.5f", translationSpeed, translation, wantedTranslation, translationStep);
    //ROS_INFO("Rotation:    v=%.5f d=%.5f - wanted=%.5f step=%.5f", rotationSpeed, rotation, wantedRotation, rotationStep);
    
    // Check if we achieved our current goal and are still moving
    if (!this->buttonFound && fabs(wantedTranslation - translation) < translationEpsilon && fabs(wantedRotation - rotation) < rotationEpsilon) {
        
	wait_start = ros::Time::now();
      
        // Set new wanted translation and rotation
        if (wantedTranslation < translationMaximum) {
            wantedTranslation += translationStep;
        }
        if (wantedTranslation > translationMaximum) {
	    wantedTranslation = translation;
            translationStep = -translationStep;
            wantedRotation += rotationStep;
        }
        if (wantedTranslation < 0.0) {
	    wantedTranslation = 0.0;
	    translationStep = -translationStep;
	    wantedRotation += rotationStep;
	}
        if (wantedRotation > rotationMaximum) {
            wantedRotation = rotationMinimum;
        }
    }    
    
    if ((ros::Time::now() - wait_start).toSec() > 1.0) {
      
      // Send new position to arm
      aidu_robotarm::robot_arm_positions arm_position;
      arm_position.translation = wantedTranslation;
      arm_position.rotation = wantedRotation;
      arm_position.extention = 0.0;
      robotArmPublisher.publish(arm_position);
      
    }
}

bool LocateButton::finished() {
    if(this->buttonFound) {
        aidu_robotarm::robot_arm_positions arm_position;
	arm_position.translation = translation;
	arm_position.rotation = rotation;
	arm_position.extention = 0.0;
	robotArmPublisher.publish(arm_position);
    }
    return this->buttonFound;
}

void LocateButton::visibleButton(const aidu_elevator::Button::ConstPtr& message) {
    ROS_INFO("Button: %d, Found: %d", message->button_type, buttonFound);
    if (message->button_type == aidu_elevator::Button::BUTTON_DOWN) {
        this->buttonFound = true;
    }
}

void LocateButton::updateArmState(const sensor_msgs::JointState::ConstPtr& message) {
    for(unsigned int i=0; i<message->name.size(); i++) {
        if (message->name[i] == "base_spindlecaddy") {
            translationSpeed = message->velocity[i];
            translation = message->position[i];
        } else if (message->name[i] == "spindlecaddy_rotationalarm") {
            rotationSpeed = message->velocity[i];
            rotation = message->position[i];
        }
    }
}







