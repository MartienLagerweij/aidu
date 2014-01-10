#include <sensor_msgs/JointState.h>
#include <aidu_robotarm/test.h>
#include <aidu_robotarm/robot_arm.h>
#include <threemxl/dxlassert.h>
#include <aidu_robotarm/motor.h>
#include <math.h>
#include <algorithm>

#define BOUND(x,y) std::max(std::min(x, y), -y)

using namespace aidu;

mobile_robot_arm::Robot_arm::Robot_arm() : core::Node::Node() {
    
    // Read parameters from config file
    std::string motor_port_name, motor_config_name;
    nh->getParam("motor_port", motor_port_name);
    nh->getParam("motor_config", motor_config_name);
    
    // Get sizes from launch parameters
    translation=rotation=extention=0.0;
    
    //creating translation, rotation and extension motor
    translationMotor = new mobile_robot_arm::Motor("translation", motor_port_name, motor_config_name);
    rotationMotor = new mobile_robot_arm::Motor("rotation", motor_port_name, motor_config_name);
    extensionMotor = new mobile_robot_arm::Motor("extension", motor_port_name, motor_config_name);
    
    // Subscribing
    test = nh->subscribe("test", 1, &mobile_robot_arm::Robot_arm::testcallback, this);
    
    // Publishing 
    joint_pub= nh->advertise<sensor_msgs::JointState>("/arm_state", 1);
  
}

void mobile_robot_arm::Robot_arm::testcallback(const aidu_robotarm::test::ConstPtr& msg){
  translation=msg->translation;
  rotation=msg->rotation;
  extention=msg->extention;
}

void mobile_robot_arm::Robot_arm::jointStatePublisher() {
    
    // Initialize
    sensor_msgs::JointState joint_state;
    
    // Get joint positions
    /*
    double translation= translationMotor->getLinearPosition();
    double rotation= rotationMotor->getLinearPosition();
    double extension= extensionMotor->getLinearPosition();
    */
    
    
    //update joint_state
    joint_state.header.stamp = ros::Time::now();
    joint_state.name.resize(3);
    joint_state.position.resize(3);
    joint_state.name[0] ="base_spindlecaddy";
    joint_state.position[0] = translation;
    joint_state.name[1] ="spindlecaddy_rotationalarm";
    joint_state.position[1] = rotation;
    joint_state.name[2] ="rotationalarm_extension";
    joint_state.position[2] = extention;
    
    // Publish the base state
    joint_pub.publish(joint_state);
    
}


void mobile_robot_arm::Robot_arm::spin(){

    ros::Rate rate(30); // rate at which position published (hertz)
    
    while(ros::ok()) {
        /*
        // Update motor information
        translationMotor->update();
        rotationMotor->update();
	extensionMotor->update();
        */
        // Publish information
        jointStatePublisher();
        
        // Spin ros
        ros::spinOnce();
        rate.sleep();

    }
}

mobile_robot_arm::Robot_arm::~Robot_arm() {
  
    delete translationMotor;
    delete rotationMotor;
    delete extensionMotor;
    
}

int main(int argc, char **argv) {
  
    ros::init(argc, argv, "robot_arm");
    mobile_robot_arm::Robot_arm robot_arm;
    robot_arm.spin();
    
    return 0;
}
