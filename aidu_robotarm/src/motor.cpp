#include <ros/ros.h>
#include <aidu_robotarm/motor.h>
#include <threemxl/C3mxlROS.h>
#include <XMLConfiguration.h>
#include <string>

using namespace aidu;

mobile_robot_arm::Motor::Motor(std::string name, std::string motor_port_name, std::string motor_config_name) {
    
    // Load motor configuration
    CXMLConfiguration motor_config_xml;
    motor_config_xml.loadFile(motor_config_name);
    
    // ThreeMXL initialization
    // Create configuration
    config = new CDxlConfig();
    config->readConfig(motor_config_xml.root().section(name.c_str()));
    this->name = name;

    // Create and configure 3mxl motor
    motor = new C3mxlROS(motor_port_name.c_str());
    motor->setConfig(config);
    motor->init();
    motor->set3MxlMode(POSITION_MODE);
    currentVelocity = 0;

}


void mobile_robot_arm::Motor::setPosition(float position) {

    // Send velocity to the 3mxl board
    ROS_INFO("%s: Setting position to %f", name.c_str(), position);
    
    // Check mode of 3mxl
    motor->get3MxlMode();
    if(motor->present3MxlMode() != POSITION_MODE){
        motor->set3MxlMode(POSITION_MODE);
    }
    
    // Send position to 3mxl
    motor->setPos(position);
}

void mobile_robot_arm::Motor::reset() {
    update();
    initialPos = motor->presentLinearPos();
}

void mobile_robot_arm::Motor::update() {
    motor->getPosAndSpeed();
    motor->getLinearPos();
}


double mobile_robot_arm::Motor::getLinearPosition() {
    return motor->presentLinearPos() - initialPos;
}

mobile_robot_arm::Motor::~Motor(){
    motor->setSpeed(0);
    serial_port->port_close();
    delete config;
    delete motor;
    delete serial_port;
}