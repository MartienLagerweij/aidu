#ifndef AIDU_MOBILE_BASE__BASE_H
#define AIDU_MOBILE_BASE__BASE_H

#include <ros/ros.h>
#include <aidu_core/node.h>
#include <aidu_mobile_base/motor.h>

namespace aidu {
  namespace mobile_base {
    class Base : public aidu::core::Node {
    public:
      
      Base();
      ~Base();
      
      // Add callbacks for velocity from differential_drive and navigation packages
      
    //protected:
      
      aidu::mobile_base::Motor* leftWheelMotor;
      aidu::mobile_base::Motor* rightWheelMotor;
      
    };
  }
}

#endif