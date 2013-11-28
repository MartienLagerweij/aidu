#ifndef AIDU_MOBILE_BASE__BASE_H
#define AIDU_MOBILE_BASE__BASE_H

#include <ros/ros.h>
#include <aidu_core/node.h>
#include <aidu_mobile_base/motor.h>
#include <aidu_mobile_base/Pos.h>
#include <geometry_msgs/Twist.h>



namespace aidu {
  namespace mobile_base {
      
    class Base : public aidu::core::Node {
    public:
      
      Base();
      ~Base();
      void speed(const geometry_msgs::Twist::ConstPtr& msg);///< sets the velocity of a motor
      void pos(const geometry_msgs::Twist::ConstPtr& msg);///< sets the positon of a motor
      void posspeed(const geometry_msgs::Twist::ConstPtr& msg); ///< sets a position using speed control
      void spin();///< reads the position of the motors
      //Vel calcVelocity(float velocity,float angle); ///< calculates the speed difference between the left and right motor

    protected:

      double radiusBase;
      double radiusWheel;
      aidu::mobile_base::Motor* leftWheelMotor;
      aidu::mobile_base::Motor* rightWheelMotor;
      ros::Subscriber possubscriber; ///< The subscriber for the position messages
      ros::Subscriber speedsubscriber; ///< The subscriber for the speed messages
      ros::Subscriber posspeedsubscriber; ///< The subscriber for the position messages that use speed mode
      ros::Publisher pospublisher; ///< The publisher for the position of the base
      ros::Publisher speedpublisher; ///< The publisher for the speed
      
      double initialLeftPos;
      double initialRightPos;
      
      void resetPos();
      double getRightPos(); 
      double getLeftPos();
      double getAngle();
      
    };
  }
}

#endif