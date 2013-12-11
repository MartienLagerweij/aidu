#ifndef AIDU_MOBILE_BASE__ODOMETRY_H
#define AIDU_MOBILE_BASE__ODOMETRY_H

#include <ros/ros.h>
#include <aidu_core/node.h>
#include <aidu_mobile_base/BaseState.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

namespace aidu {
    namespace mobile_base {
        
        class Odometry : public aidu::core::Node {
        public:
        
            Odometry();
            ~Odometry();
            void processState(const aidu_mobile_base::BaseState::ConstPtr& state); ///< Converts state to odometry
            
        protected:
            
            ros::Subscriber stateSubscriber; ///< The subscriber for the states
            ros::Publisher odometryPublisher; ///< The publisher for the odometry
            tf::TransformBroadcaster odometryBroadcaster; ///< The odometry tf broadcaster
            
            double x; ///< The current x position
            double y; ///< The current y position
            
            double previousLeftWheelPosition;
            double previousRightWheelPosition;
        
        };
    }
}

#endif