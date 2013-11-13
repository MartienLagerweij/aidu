#include <ros/ros.h>
#include <aidu_mobile_base/motor.h>
#include <threemxl/C3mxlROS.h>

using namespace aidu;

mobile_base::Motor::Motor(int id) : core::Node::Node() {
  
  // ThreeMXL initialization
    // Create configuration
    config = new CDxlConfig();

    // Open serial port
    serial_port = new LxSerial();
    serial_port->port_open("/dev/ttyUSB0", LxSerial::RS485_FTDI);
    serial_port->set_speed(LxSerial::S921600);

    // Create 3mxl motor
    motor = new C3mxl();

    // Set parameters for motor
    motor->setSerialPort(serial_port);
    motor->setConfig(config->setID(id));
    motor->init(false);
    motor->set3MxlMode(SPEED_MODE);
}

void mobile_base::Motor::setVelocity(float velocity) {
  
  // Send velocity to the 3mxl board
  printf("Setting velocity to %f\n", velocity);
  motor->setLinearSpeed(velocity);
}

  

mobile_base::Motor::~Motor(){
    motor->setSpeed(0);
    serial_port->port_close();
    delete config;
    delete motor;
    delete serial_port;
    ROS_INFO("mobile_base");
}
