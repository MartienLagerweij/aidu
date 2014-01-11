#ifndef AIDU_ELEVATOR__ACTIONS__LOCATEBUTTON_H
#define AIDU_ELEVATOR__ACTIONS__LOCATEBUTTON_H

#include <ros/ros.h>
#include <aidu_elevator/actions/action.h>
#include <aidu_elevator/Button.h>

namespace aidu {
    namespace elevator {
        class LocateButton : public aidu::elevator::Action {
            public:
            
                LocateButton(ros::NodeHandle* nh);
                ~LocateButton();
                void execute();
                bool finished();
                
                void visibleButton(const aidu_elevator::Button::ConstPtr& message);
                
            protected:
                bool buttonFound;
                ros::Subscriber buttonSubscriber;
        
        };
    }
}

#endif