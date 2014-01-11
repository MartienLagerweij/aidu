#ifndef AIDU_ELEVATOR__ELEVATOR_H
#define AIDU_ELEVATOR__ELEVATOR_H

#include <ros/ros.h>
#include <aidu_core/node.h>
#include <aidu_elevator/actions/action.h>
#include <aidu_elevator/OutsideButton.h>
#include <aidu_elevator/ElevatorNav.h>

namespace aidu {
   class Elevator : public aidu::core::Node {
        public:
        
            Elevator();
            void spin();
	    
	protected:
            void setupActions();
            aidu::elevator::Action* currentAction;
            
   };
}

#endif