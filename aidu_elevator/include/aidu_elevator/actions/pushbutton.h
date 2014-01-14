#ifndef AIDU_ELEVATOR__ACTIONS__PUSHBUTTON_H
#define AIDU_ELEVATOR__ACTIONS__PUSHBUTTON_H

#include <ros/ros.h>
#include <aidu_elevator/actions/action.h>
#include <aidu_elevator/Button.h>

namespace aidu {
    namespace elevator {
        class PushButton : public aidu::elevator::Action {
            public:
            
                PushButton(ros::NodeHandle* nh);
                ~PushButton();
                void execute();
                bool finished();
                
                void visibleButton(const aidu_elevator::Button::ConstPtr& message);
                
            protected:
                bool buttonPushed;
                ros::Subscriber buttonSubscriber;
		ros::Publisher robot_arm_positions_pub;
        
        };
    }
}

#endif