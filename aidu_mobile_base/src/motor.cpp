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
    modepos=true;
    


}

void mobile_base::Motor::setVelocity(float velocity) {
  
  // Send velocity to the 3mxl board
  ROS_INFO("%s: Setting velocity to %f", name, velocity);
  
  //checking mode of 3mxl
  if(modepos){
    motor->set3MxlMode(SPEED_MODE);
    modepos=false;
  }
  //sending speed to 3mxl
  motor->setSpeed(velocity);
  
  /*
  // Debug information
  ROS_INFO("error: %d", motor->getLastError());
  motor->get3MxlMode();
  ROS_INFO("mode: %d", motor->present3MxlMode());
  double p, d, i, il = -1;
  motor->getPIDSpeed(p, d, i, il);
  ROS_INFO("P,D,I,IL = %f, %f, %f, %f", p, d, i, il);
  */
}

void mobile_base::Motor::setPosition(float position) {

  // Send velocity to the 3mxl board
  ROS_INFO("Setting position to %f", position);

  //Checking mode of 3mxl
  if(!modepos){
    motor->set3MxlMode(POSITION_MODE);
    modepos=true;
  }
  // sending position to 3mxl
  motor->setPos(position, 0.2, 1.0);
}


mobile_base::Motor::~Motor(){
    motor->setSpeed(0);
    serial_port->port_close();
    delete config;
    delete motor;
    delete serial_port;
}
