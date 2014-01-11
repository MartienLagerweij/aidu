#ifndef AIDU_ELEVATOR__ACTIONS__ACTION_H
#define AIDU_ELEVATOR__ACTIONS__ACTION_H

#include <ros/ros.h>

namespace aidu {
    namespace elevator {
        class Action {
            public:
            
                Action(ros::NodeHandle* nh);
                virtual ~Action();
                virtual void execute();
                virtual bool finished();
                
                void setNextAction(aidu::elevator::Action* action);
                aidu::elevator::Action* getNextAction();
                
            protected:
                aidu::elevator::Action* nextAction;
                    
        };
    }
}

#endif