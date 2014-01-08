#ifndef AIDU_ELEVATOR__OUTSIDE_H
#define AIDU_ELEVATOR__OUTSIDE_H

#include <ros/ros.h>
#include <aidu_core/node.h>
#include <aidu_elevator/OutsideButton.h>

namespace aidu {
   class OutsideButton : public aidu::core::Node {
        public:
        
	OutsideButton();
	    
	protected:
	    ros::Subscriber pushoutsidebuttonSubsciber; ///< Push outside button when message received
            void OutsideButtonCallback(const aidu_elevator::OutsideButton::ConstPtr& outside);
   };
}

#endif