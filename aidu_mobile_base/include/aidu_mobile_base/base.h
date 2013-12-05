#ifndef AIDU_MOBILE_BASE__BASE_H
#define AIDU_MOBILE_BASE__BASE_H

#include <ros/ros.h>
#include <aidu_core/node.h>
#include <aidu_mobile_base/motor.h>
#include <aidu_mobile_base/Pos.h>
#include <aidu_mobile_base/Config.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>

namespace aidu {
    namespace mobile_base {
        
        class Base : public aidu::core::Node {
        public:
        
            Base();
            ~Base();
            
            void speed(const geometry_msgs::Twist::ConstPtr& msg); ///< sets the velocity of a motor
            void position(const geometry_msgs::Twist::ConstPtr& msg); ///< sets the positon of a motor
            void positionSpeed(const geometry_msgs::Twist::ConstPtr& msg); ///< sets a position using speed control
            void setConfig(const aidu_mobile_base::Config::ConstPtr& msg); ///< Sets the configuration
            void spin(); ///< reads the position of the motors
            void leftWheelSpeed(const std_msgs::Float32::ConstPtr& msg);
            void rightWheelSpeed(const std_msgs::Float32::ConstPtr& msg);

        protected:

            double radiusBase;
            double radiusWheel;
            
            aidu::mobile_base::Motor* leftWheelMotor;
            aidu::mobile_base::Motor* rightWheelMotor;
            
            ros::Subscriber positionSubscriber; ///< The subscriber for the position messages
            ros::Subscriber speedSubscriber; ///< The subscriber for the speed messages
            ros::Subscriber positionSpeedSubscriber; ///< The subscriber for the position messages that use speed mode
            ros::Subscriber configSubscriber; ///< The subscriber for the base configuration
            ros::Subscriber leftWheelSubscriber; ///< The subscriber for the left wheel target velocities
            ros::Subscriber rightWheelSubscriber; ///< The subscriber for the right wheel target velocities
            
            ros::Publisher statePublisher; ///< The publisher for the position of the base
            ros::Publisher speedPublisher; ///< The publisher for the speed
            
            double initialLeftPos;
            double initialRightPos;
            
            double maximumLinearVelocity;
            double maximumAngularVelocity;
            
            double currentX;
            double currentY;
            double currentTheta;
            
            void resetPos();
            double getRightPos(); 
            double getLeftPos();
            double getAngle();
            double getLinearVelocity();
            double getAngularVelocity();
            
            void publishState();
            void publishOdometry();
        
        };
    }
}

#endif