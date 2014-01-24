#ifndef AIDU_ELEVATOR__ELEVATOR_H
#define AIDU_ELEVATOR__ELEVATOR_H

#include <ros/ros.h>
#include <aidu_core/node.h>
#include <aidu_elevator/actions/action.h>
#include <aidu_elevator/OutsideButton.h>
#include <aidu_elevator/ElevatorNavigation.h>

namespace aidu {
   class Elevator : public aidu::core::Node {
        public:
        
            Elevator();
            void spin();
            void activate(const aidu_elevator::ElevatorNavigation::ConstPtr& message);
	    
	protected:
            void setupActions();
            void removeActions();
            aidu::elevator::Action* currentAction;
            ros::Subscriber subscriber;
            
            int targetFloor;
            int currentFloor;
            
   };
}

#endif