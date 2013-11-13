
#include <aidu_mobile_base/base.h>

using namespace aidu;

mobile_base::Base::Base() : core::Node::Node() {
  leftWheelMotor = new mobile_base::Motor(106);
  rightWheelMotor = new mobile_base::Motor(107);
}

mobile_base::Base::~Base() {
  delete leftWheelMotor;
  delete rightWheelMotor;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "motor");
  mobile_base::Base base;

  int frequency = 10;
  ros::Rate loop_rate(frequency);
  int count = 0;
  int seconds = 4;
  while(ros::ok() && count < frequency*seconds) {
    base.leftWheelMotor->setVelocity(2); // Test left wheel velocity!
    base.rightWheelMotor->setVelocity(2); // Test right wheel velocity!
    loop_rate.sleep();
    count++;
  }

  return 0;
}
