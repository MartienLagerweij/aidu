#ifndef AIDU_MOBILE_BASE__MOTOR_H
#define AIDU_MOBILE_BASE__MOTOR_H

#include <ros/ros.h>
#include <aidu_core/node.h>

namespace aidu {
  namespace mobile_base {
    class Motor : public aidu::core::Node {
    public:
      
      // Add proper variables for threemxl (remember: also edit motor.cpp with appropriate initialization in the constructor!)
      // See Hand.h and Hand.cpp in gripper example, this will be very similar (ignore the subscriber/publisher in there)
      // We need stuff like CDxlGeneric *motor, config and serial_port.
      
      Motor(int id);
      
    //protected:
      
      void setVelocity(float velocity);
      
    };
  }
}

#endif