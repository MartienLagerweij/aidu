
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

 ros::Rate loop_rate(10);
    while(ros::ok()) {

    base.leftWheelMotor->setVelocity(5); // Test left wheel velocity!
    base.rightWheelMotor->setVelocity(-15); // Test right wheel velocity!
    loop_rate.sleep();
}

  return 0;
}
