#ifndef AIDU_MOBILE_BASE__MOTOR_H
#define AIDU_MOBILE_BASE__MOTOR_H

#include <ros/ros.h>
#include <threemxl/C3mxlROS.h>
#include <CDxlGeneric.h>
#include <CDxlGroup.h>
#include <string>

namespace aidu {
  namespace mobile_base {
    class Motor {
    public:
      
      Motor(std::string name, std::string motor_port_name, std::string motor_config_name);
      std::string name;
      ~Motor();
      void setVelocity(float velocity); ///< Sets the velocity of the motor
      void setPosition(float position);///< Sets the position of the motor
      CDxlGeneric *motor;               ///< The motor interface
      
    protected:
      CDxlConfig *config;               ///< The motor configuration
      LxSerial* serial_port;            ///< The serial port
      float currentvel;
      
    };
  }
}

#endif
