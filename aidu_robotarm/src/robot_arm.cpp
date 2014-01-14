#include <sensor_msgs/JointState.h>
#include <aidu_robotarm/robot_arm_positions.h>
#include <aidu_robotarm/robot_arm.h>
#include <threemxl/dxlassert.h>
#include <aidu_robotarm/motor.h>


#define BOUND(x,y) std::max(std::min(x, y), 0.f)

using namespace aidu;

mobile_robot_arm::Robot_arm::Robot_arm() : core::Node::Node() {
    
    // Read parameters from config file
    std::string motor_port_name, motor_config_name;
    nh->getParam("motor_port", motor_port_name);
    nh->getParam("motor_config", motor_config_name);
    
    // initialising variables
    current_translation=current_rotation=current_extention=0.0;
    target_translation=target_rotation=target_extention=0.0;
    max_translation=0.45;
    max_rotation=3.1415;
    max_extention=0.075;
    
    //creating translation, rotation and extension motor
    translationMotor = new mobile_robot_arm::Motor("translation", motor_port_name, motor_config_name);
    rotationMotor = new mobile_robot_arm::Motor("rotation", motor_port_name, motor_config_name);
    extensionMotor = new mobile_robot_arm::Motor("extension", motor_port_name, motor_config_name);
    
    // Subscribing
    position_sub = nh->subscribe("/robot_arm_positions", 1, &mobile_robot_arm::Robot_arm::positioncallback, this);
    
    // Publishing 
    joint_pub= nh->advertise<sensor_msgs::JointState>("/arm_state", 1);
  
}

void mobile_robot_arm::Robot_arm::positioncallback(const aidu_robotarm::robot_arm_positions::ConstPtr& msg){
  target_translation=BOUND(msg->translation,max_translation);
  target_rotation=BOUND(msg->rotation,max_rotation);
  target_extention=BOUND(msg->extention,max_extention);
}

bool mobile_robot_arm::Robot_arm::setPos(){
  
  bool trans_pos=fabs(target_translation - current_translation) < 0.01;
  bool rot_pos=fabs(target_rotation - current_rotation) < 0.01;
  bool ext_pos=fabs(target_extention - current_extention) < 0.01;
  bool total_pos=false;
  
  // set positions in correct order
  if (!trans_pos){
    translationMotor->setPosition(target_translation);
  } else if (!rot_pos){
    rotationMotor->setPosition(target_rotation);
  } else if (!ext_pos){
    extensionMotor->setPosition(target_extention);
  }
  // check if position attained
  if (trans_pos && rot_pos && ext_pos){
    total_pos=true;
  }
  
  return total_pos;
}

void mobile_robot_arm::Robot_arm::jointStatePublisher() {
    
    // Initialize
    sensor_msgs::JointState joint_state;
    
    // Get joint positions
    current_translation= translationMotor->getLinearPosition();
    current_rotation= rotationMotor->getLinearPosition();
    current_extention= extensionMotor->getLinearPosition();
    
    //update joint_state
    joint_state.header.stamp = ros::Time::now();
    joint_state.name.resize(3);
    joint_state.position.resize(3);
    joint_state.name[0] ="base_spindlecaddy";
    joint_state.position[0] = current_translation;
    joint_state.name[1] ="spindlecaddy_rotationalarm";
    joint_state.position[1] = current_rotation;
    joint_state.name[2] ="rotationalarm_extension";
    joint_state.position[2] = current_extention;
    
    // Publish the joint state
    joint_pub.publish(joint_state);    
}


void mobile_robot_arm::Robot_arm::spin(){

    ros::Rate rate(30); // rate at which position published (hertz)
    
    while(ros::ok()) {
        
        // Update motor information
        translationMotor->update();
        rotationMotor->update();
	extensionMotor->update();
	
        // Publish joint information
        jointStatePublisher();
        
	//move robot_arm
	bool pos_attained=setPos();
	
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
