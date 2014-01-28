#ifndef AIDU_ELEVATOR__ACTIONS__REPOSITION_H
#define AIDU_ELEVATOR__ACTIONS__REPOSITION_H

#include <ros/ros.h>
#include <aidu_elevator/actions/action.h>
#include <aidu_vision/DistanceSensors.h>


namespace aidu {
    namespace elevator {
        class Reposition : public aidu::elevator::Action {
            public:
            
                Reposition(ros::NodeHandle* nh);
                ~Reposition();
                void execute();
                bool finished();
                
            protected:
                bool done;
                ros::Publisher positionPublisher;
                void moveBase(double linear, double angular);
        
        };
    }
}

#endif