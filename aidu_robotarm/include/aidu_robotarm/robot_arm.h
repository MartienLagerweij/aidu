#ifndef AIDU_MOBILE_ROBOT_ARM__ROBOT_ARM_H
#define AIDU_MOBILE_ROBOT_ARM__ROBOT_ARM_H

#include <ros/ros.h>
#include <aidu_core/node.h>
#include <aidu_robotarm/motor.h>
#include <aidu_robotarm/robot_arm_positions.h>

namespace aidu {
    namespace mobile_robot_arm {
        
        class Robot_arm : public aidu::core::Node {
        public:
        
            Robot_arm();
            ~Robot_arm();
            
            void spin(); ///< reads the position of the motors
	    void positioncallback(const aidu_robotarm::robot_arm_positions::ConstPtr& msg);

        protected:
            
            aidu::mobile_robot_arm::Motor* translationMotor;
            aidu::mobile_robot_arm::Motor* rotationMotor;
            aidu::mobile_robot_arm::Motor* extensionMotor;
	    
	    double current_translation,current_rotation,current_extention; // joint states
	    double target_translation,target_rotation,target_extention; // target joint states
	    float max_translation,max_rotation,max_extention; // target joint states
	    
	    ros::Publisher joint_pub;
	    ros::Subscriber position_sub;
	    
            
            void resetPos();
	    bool setPos();
            double getLinearVelocity();
            double getAngularVelocity();
            
            void jointStatePublisher();
        
        };
    }
}

#endif