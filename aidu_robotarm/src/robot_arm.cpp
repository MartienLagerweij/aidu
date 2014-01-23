#include <sensor_msgs/JointState.h>
#include <aidu_robotarm/robot_arm_positions.h>
#include <aidu_robotarm/robot_arm.h>
#include <threemxl/dxlassert.h>
#include <aidu_robotarm/motor.h>


#define BOUND(x,y,z) std::max(std::min(x, y), z)

using namespace aidu;

mobile_robot_arm::Robot_arm::Robot_arm() : core::Node::Node() {
    
    // Read parameters from config file
    std::string motor_port_name, motor_config_name;
    nh->getParam("motor_port", motor_port_name);
    nh->getParam("motor_config", motor_config_name);
    
    // initialising variables
    current_translation=current_rotation=current_extention=0.0;
    target_translation=target_rotation=target_extention=0.0;
    max_translation=370; min_translation=0.0;
    max_rotation=3.1415; min_rotation=-1.57;
    max_extention=75; min_extention=0.0;
    
    //creating translation, rotation and extension motor 
    extensionMotor = new mobile_robot_arm::Motor("extention", motor_port_name, motor_config_name);
    translationMotor = new mobile_robot_arm::Motor("translation", motor_port_name, motor_config_name);
    rotationMotor = new mobile_robot_arm::Motor("rotation", motor_port_name, motor_config_name);
    
    //initialising positions of motors
    extensionMotor->initialize(-0.5,-1.0);
    translationMotor->initialize(-7.0,-1.0);
    rotationMotor->initialize(1.0,1.0);
    // Subscribing
    position_sub = nh->subscribe("/robot_arm_positions", 1, &mobile_robot_arm::Robot_arm::positioncallback, this);
    
    // Publishing 
    joint_pub= nh->advertise<sensor_msgs::JointState>("/arm_state", 1);
  
}

void mobile_robot_arm::Robot_arm::positioncallback(const aidu_robotarm::robot_arm_positions::ConstPtr& msg){
  target_translation=BOUND(msg->translation,max_translation,min_translation);
  target_rotation=BOUND(msg->rotation,max_rotation,min_rotation);
  target_extention=BOUND(msg->extention,max_extention,min_extention);
}

bool mobile_robot_arm::Robot_arm::setPos(){
  
  bool trans_pos=fabs(target_translation - current_translation) < 0.005;
  bool rot_pos=fabs(target_rotation - current_rotation) < 0.05;
  bool ext_pos=fabs(target_extention - current_extention) < 0.05;

  ROS_INFO("trans_pos :%d  rot_pos: %d ext_pos: %d",trans_pos,rot_pos,ext_pos);
  
  // set positions in correct order
  if (!trans_pos){
    translationMotor->setLinearPosition(target_translation,0.01);
  } else if (!rot_pos){
    rotationMotor->setPosition(target_rotation,1.5);
  } else if (!ext_pos){
    extensionMotor->setPosition(target_extention,1.0);
  }
  // check if position attained
  return(trans_pos && rot_pos && ext_pos);

  
  
}

void mobile_robot_arm::Robot_arm::jointStatePublisher() {
    
    // Initialize
    sensor_msgs::JointState joint_state;
    
    // Get joint positions
    current_translation= translationMotor->getLinearPosition();
    current_rotation= rotationMotor->getPosition();
    current_extention= extensionMotor->getPosition();
    ROS_INFO("translation:[%f , %f]   rotation:[%f , %f]  extension:[%f , %f] ",current_translation,target_translation, current_rotation,target_rotation, current_extention, target_extention);
    
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

    ros::Rate rate(20); // rate at which position published (hertz)
    
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
