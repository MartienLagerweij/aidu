#ifndef AIDU_ELEVATOR__ELEVATOR_H
#define AIDU_ELEVATOR__ELEVATOR_H

#include <ros/ros.h>
#include <aidu_core/node.h>
#include <aidu_elevator/OutsideButton.h>
#include <aidu_elevator/ElevatorNav.h>

namespace aidu {
   class Elevator : public aidu::core::Node {
        public:
        
            Elevator();
	    
	protected:
	    ros::Subscriber elevatorSubscriber; ///< The main subscriber starting the elevator proces
            ros::Publisher pushoutsidebuttonPubliher; ///< The subscriber for the speed messages
            void initiateCallback(const aidu_elevator::ElevatorNav::ConstPtr& elevator_initiate);
   };
}

#endif