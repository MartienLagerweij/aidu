#ifndef AIDU_ELEVATOR__ACTIONS__MOVETOBUTTON_H
#define AIDU_ELEVATOR__ACTIONS__MOVETOBUTTON_H

#include <ros/ros.h>
#include <aidu_elevator/actions/action.h>
#include <aidu_vision/DistanceSensors.h>
#include <geometry_msgs/Twist.h>


namespace aidu {
    namespace elevator {
        class MoveToButton : public aidu::elevator::Action {
            public:
            
                MoveToButton(ros::NodeHandle* nh, int button);
                ~MoveToButton();
                void execute();
                bool finished();
                
                void sensorCallback(const aidu_vision::DistanceSensors::ConstPtr& dist_msg);
                void buttonCallback(const aidu_elevator::Button::ConstPtr& message);
                void moveBase(double linear, double angular);
                
            protected:
                ros::Subscriber sensorSubscriber;
                ros::Subscriber buttonSubscriber;
                ros::Publisher positionPublisher;
                double distance;
                double buttonX;
                bool inFrontOfButton;
                int button;
                
        
        };
    }
}

#endif