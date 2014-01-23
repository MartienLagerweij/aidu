#ifndef AIDU_ELEVATOR__ACTIONS__LOCATEBUTTON_H
#define AIDU_ELEVATOR__ACTIONS__LOCATEBUTTON_H

#include <ros/ros.h>
#include <ros/sensor_msgs/JointState.h>
#include <aidu_elevator/actions/action.h>
#include <aidu_elevator/Button.h>

extern std::pair< C::iterator, C::iterator > r;
namespace aidu {
    namespace elevator {
        class LocateButton : public aidu::elevator::Action {
            public:
            
                LocateButton(ros::NodeHandle* nh);
                ~LocateButton();
                void execute();
                bool finished();
                
                void visibleButton(const aidu_elevator::Button::ConstPtr& message);
                void updateArmState(const sensor_msgs::JointState::ConstPtr& message);
                
            protected:
                bool buttonFound;
                
                double translation;
                double rotation;
                
                double translationSpeed;
                double rotationSpeed;
                
                double wantedTranslation;
                double wantedRotation;
                
                double translationStep;
                double rotationStep;
                
                double translationEpsilon;
                double rotationEpsilon;
                
                double translationMaximum;
                double translationMinimum;
                
                double rotationMaximum;
                double rotationMinimum;
                
                ros::Subscriber buttonSubscriber;
                ros::Subscriber robotArmSubscriber;
		ros::Publisher robotArmPublisher;
        
        };
    }
}

#endif