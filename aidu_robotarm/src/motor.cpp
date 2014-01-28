#include <ros/ros.h>
#include <aidu_robotarm/motor.h>
#include <threemxl/C3mxlROS.h>
#include <XMLConfiguration.h>
#include <3mxlControlTable.h>
#include <string>
#include <boost/iterator/iterator_concepts.hpp>

using namespace aidu;

mobile_robot_arm::Motor::Motor(std::string name, std::string motor_port_name, std::string motor_config_name) {
    
    ROS_INFO("creating %s",name.c_str());
    // Load motor configuration
    CXMLConfiguration motor_config_xml;
    motor_config_xml.loadFile(motor_config_name);
    
    // ThreeMXL initialization
    // Create configuration
    config = new CDxlConfig();
    config->readConfig(motor_config_xml.root().section(name.c_str()));
    

    // Create and configure 3mxl motor
    motor = new C3mxlROS(motor_port_name.c_str());
    motor->setConfig(config);
    int init=-20;
    while (init != 0 && ros::ok()){
      init=motor->init();
      sleep(1);
      //ROS_INFO("init: %d",init);
    }
    
    this->name = name;
    motor->setMaxPeakMotorCurrent(9.0);
    
}

void mobile_robot_arm::Motor::initialize(double speed, double torque){
    // Initialize motor physically
    motor->set3MxlMode(EXTERNAL_INIT);
    ros::Rate looprate(5);
    motor->setAcceleration(1.5);
    motor->setSpeed(speed);
    motor->setTorque(torque);
    while(ros::ok() && motor->presentStatus() != M3XL_STATUS_INIT_DONE) {
        looprate.sleep();
	motor->getStatus();
    }
    ROS_INFO("%s initial position atained",this->name.c_str());
    motor->setSpeed(0.0);
    motor->setTorque(0.0);
    currentVelocity = 0;
}


void mobile_robot_arm::Motor::setPosition(float position, float speed) {

    // Check mode of 3mxl
    motor->get3MxlMode();
    if(motor->present3MxlMode() != POSITION_MODE){
        motor->set3MxlMode(POSITION_MODE);
    }
    
    // Send position to 3mxl
    ROS_INFO("present pos: %f",motor->presentPos());
    if (fabs(position-motor->presentPos())>0.001){
      ROS_INFO("%s: Setting position to %f", name.c_str(), position);
      motor->setPos(position,speed);
    }
}
    
void mobile_robot_arm::Motor::setLinearPosition(float position, float speed){
    // Check mode of 3mxl
    motor->get3MxlMode();
    if(motor->present3MxlMode() != POSITION_MODE){
        motor->set3MxlMode(POSITION_MODE);
    }
    
    // Send position to 3mxl
    ROS_INFO("present pos: %f",motor->presentLinearPos());
    if (fabs(position-motor->presentLinearPos())>0.001){
      ROS_INFO("%s: Setting position to %f", name.c_str(), position);
      motor->setLinearPos(position,speed);
    }
       
}

void mobile_robot_arm::Motor::reset() {
    update();
    initialPos = motor->presentPos();
}

void mobile_robot_arm::Motor::update() {
    motor->getPosAndSpeed();
    motor->getLinearPos();
}


double mobile_robot_arm::Motor::getPosition() {
    return motor->presentPos();
}

double mobile_robot_arm::Motor::getLinearPosition() {
    return motor->presentLinearPos();
}

double mobile_robot_arm::Motor::getSpeed(){
  return motor->presentSpeed();
}

mobile_robot_arm::Motor::~Motor(){
    motor->setSpeed(0);
    serial_port->port_close();
    delete config;
    delete motor;
    delete serial_port;
}
