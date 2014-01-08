#ifndef AIDU_ROBOT_ARM__MOTOR_H
#define AIDU_ROBOT_ARM__MOTOR_H

#include <ros/ros.h>
#include <threemxl/C3mxlROS.h>
#include <CDxlGeneric.h>
#include <CDxlGroup.h>
#include <string>

namespace aidu {
  namespace mobile_robot_arm {
    class Motor {
    public:
      
      Motor(std::string name, std::string motor_port_name, std::string motor_config_name);
      std::string name;
      ~Motor();
      void setPosition(float position);///< Sets the position of the motor
      CDxlGeneric *motor;               ///< The motor interface
      float currentVelocity;

      double initialPos;
      
      void reset();
      void update();
      double getLinearPosition();
      
    protected:
      CDxlConfig *config;               ///< The motor configuration
      LxSerial* serial_port;            ///< The serial port
      
    };
  }
}

#endif
