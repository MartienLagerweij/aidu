
#include <aidu_mobile_base/base.h>

using namespace aidu;

mobile_base::Base::Base() : core::Node::Node() {
  leftWheelMotor = new mobile_base::Motor(108);
  rightWheelMotor = new mobile_base::Motor(109);
}

mobile_base::Base::~Base() {
  delete leftWheelMotor;
  delete rightWheelMotor;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "motor");
  mobile_base::Base base;
  
  base.leftWheelMotor->setVelocity(10.0); // Test left wheel velocity!
  
  base.spin();
  return 0;
}