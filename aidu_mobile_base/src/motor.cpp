#include <ros/ros.h>
#include <aidu_mobile_base/motor.h>
#include <threemxl/C3mxlROS.h>
#include <XMLConfiguration.h>
#include <string>

using namespace aidu;

mobile_base::Motor::Motor(char* name, std::string motor_port_name, std::string motor_config_name) {
    
    // Load motor configuration
    CXMLConfiguration motor_config_xml;
    motor_config_xml.loadFile(motor_config_name);
    
    // ThreeMXL initialization
    // Create configuration
    config = new CDxlConfig();
    config->readConfig(motor_config_xml.root().section(name));
    
    this->name = name;

    // Create and configure 3mxl motor
    motor = new C3mxlROS(motor_port_name.c_str());
    motor->setConfig(config);
    motor->init();
    motor->set3MxlMode(POSITION_MODE);
    motor->setAcceleration(2.8);
    currentvel=0;

}

void mobile_base::Motor::setVelocity(float velocity) {
  if (fabs(velocity-currentvel) > 0.01) {
    currentvel = velocity;
    
    // Send velocity to the 3mxl board
    ROS_INFO("%s: Setting velocity to %f", name, velocity);
    
    // Check mode of 3mxl
    motor->get3MxlMode();
    if(motor->present3MxlMode() != SPEED_MODE){
      motor->set3MxlMode(SPEED_MODE);
    }
    
    // Send speed to 3mxl
    motor->setSpeed(velocity);
  }
}

void mobile_base::Motor::setPosition(float position) {

  // Send velocity to the 3mxl board
  ROS_INFO("%s: Setting position to %f", name, position);
  
  // Check mode of 3mxl
  motor->get3MxlMode();
  if(motor->present3MxlMode() != POSITION_MODE){
    motor->set3MxlMode(POSITION_MODE);
  }
  
  // Send position to 3mxl
  motor->setPos(position);
}


mobile_base::Motor::~Motor(){
    motor->setSpeed(0);
    serial_port->port_close();
    delete config;
    delete motor;
    delete serial_port;
}
