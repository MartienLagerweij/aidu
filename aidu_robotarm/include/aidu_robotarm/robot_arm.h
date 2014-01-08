#ifndef AIDU_MOBILE_ROBOT_ARM__ROBOT_ARM_H
#define AIDU_MOBILE_ROBOT_ARM__ROBOT_ARM_H

#include <ros/ros.h>
#include <aidu_core/node.h>
#include <aidu_robotarm/motor.h>
#include <aidu_robotarm/test.h>

namespace aidu {
    namespace mobile_robot_arm {
        
        class Robot_arm : public aidu::core::Node {
        public:
        
            Robot_arm();
            ~Robot_arm();
            
            void spin(); ///< reads the position of the motors
	    void testcallback(const aidu_robotarm::test::ConstPtr& msg);

        protected:
            
            aidu::mobile_robot_arm::Motor* translationMotor;
            aidu::mobile_robot_arm::Motor* rotationMotor;
            aidu::mobile_robot_arm::Motor* extensionMotor;
	    
	    double translation,rotation,extention;
	    
	    ros::Publisher joint_pub;
	    ros::Subscriber test;
	    
            
            void resetPos();
            double getLinearVelocity();
            double getAngularVelocity();
            
            void jointStatePublisher();
        
        };
    }
}

#endif