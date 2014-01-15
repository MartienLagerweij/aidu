#ifndef AIDU_ELEVATOR__ACTIONS__GO_TO_DOOR_H
#define AIDU_ELEVATOR__ACTIONS__GO_TO_DOOR_H

#include <ros/ros.h>
#include <aidu_elevator/actions/action.h>
#include <aidu_vision/DistanceSensors.h>


namespace aidu {
    namespace elevator {
        class GoToDoor : public aidu::elevator::Action {
            public:
            
                GoToDoor(ros::NodeHandle* nh);
                ~GoToDoor();
                void execute();
                bool finished();
                
                
            protected:
                ros::Subscriber sensor_sub;
		ros::Publisher position_pub;
		void sensorcallback(const aidu_vision::DistanceSensors::ConstPtr& dist_msg);
		bool door_open;
		double distance;
		
        
        };
    }
}

#endif