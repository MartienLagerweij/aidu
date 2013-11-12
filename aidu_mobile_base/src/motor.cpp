#include <ros/ros.h>
#include <aidu_mobile_base/motor.h>

using namespace aidu;

mobile_base::Motor::Motor(int id) : core::Node::Node() {
  
  // ThreeMXL initialization here (use the ID that is passed as an argument to identify the motor on 3mxl)
  
}

void mobile_base::Motor::setVelocity(float velocity) {
  
  // Send velocity to the 3mxl board
  printf("Setting velocity to %f\n", velocity);
  
}

