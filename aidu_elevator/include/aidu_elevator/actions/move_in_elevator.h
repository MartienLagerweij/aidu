#ifndef AIDU_ELEVATOR__ACTIONS__MOVEINELEVATOR_H
#define AIDU_ELEVATOR__ACTIONS__MOVEINELEVATOR_H

#include <ros/ros.h>
#include <aidu_elevator/actions/action.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

namespace aidu {
    namespace elevator {
        class MoveInElevator : public aidu::elevator::Action {
            public:
            
		MoveInElevator(ros::NodeHandle* nh);
		~MoveInElevator();
                void execute();
                bool finished();
                
		void laserscancallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg);
                
            protected:                
		ros::Publisher basepositionPublisher;
		ros::Subscriber laserscanSubsciber;
		int rotationdirection;
        
        };
    }
}

#endif