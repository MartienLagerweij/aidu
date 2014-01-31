#ifndef AIDU_ELEVATOR__ACTIONS__MOVE_OUT_ELEVATOR_H
#define AIDU_ELEVATOR__ACTIONS__MOVE_OUT_ELEVATOR_H

#include <ros/ros.h>
#include <aidu_elevator/actions/action.h>
#include <aidu_vision/DistanceSensors.h>
#include <sensor_msgs/JointState.h>


namespace aidu {
    namespace elevator {
        class MoveOutElevator : public aidu::elevator::Action {
            public:
            
                MoveOutElevator(ros::NodeHandle* nh);
                ~MoveOutElevator();
                void execute();
                bool finished();
                
            protected:
                bool done;
                ros::Publisher positionPublisher;
		ros::Publisher armPublisher;
		ros::Subscriber distanceSubscriber;
		double distance;
                void moveBase(double linear, double angular);
		void distanceCallback(const aidu_vision::DistanceSensors::ConstPtr& dist_msg);
        };
    }
}

#endif