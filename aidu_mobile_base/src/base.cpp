
#include <aidu_mobile_base/base.h>
#include <aidu_mobile_base/Pos.h>
#include <geometry_msgs/Twist.h>
#include <aidu_mobile_base/State.h>
#include <threemxl/dxlassert.h>

using namespace aidu;

mobile_base::Base::Base() : core::Node::Node() {
  
    // Read parameters from config file
    std::string motor_port_name, motor_config_name;
    
    nh->getParam("motor_port", motor_port_name);
    nh->getParam("motor_config", motor_config_name);
    
    //creating left and right motor
    leftWheelMotor = new mobile_base::Motor("left", motor_port_name, motor_config_name);
    rightWheelMotor = new mobile_base::Motor("right", motor_port_name, motor_config_name);

    //subscibing and pubishing position and speed topics
    possubscriber = nh->subscribe("pos", 1, &mobile_base::Base::pos, this);
    speedsubscriber = nh->subscribe("speed", 1, &mobile_base::Base::speed, this);
    pospublisher = nh->advertise<aidu_mobile_base::State>("state",1);
    
    // Get sizes from launch parameters
    nh->getParam("wheel_base", radiusBase);
    radiusBase *= 0.5;
    nh->getParam("wheel_diameter", radiusWheel);
    radiusWheel *= 0.5;
  
}

void mobile_base::Base::pos(const geometry_msgs::Twist::ConstPtr& msg) {
  
    // Reading position from topic
    float position=msg->linear.x;
    float angle=msg->angular.z;
    
    // Getting position of the wheels
    leftWheelMotor->motor->getPos();
    float posleft = leftWheelMotor->motor->presentPos();
    rightWheelMotor->motor->getPos();
    float posright = rightWheelMotor->motor->presentPos();
    
    // Compute position-based wheel position
    position = position / radiusWheel;
    
    // Compute angle-based wheel position
    float wheelDifference = angle * (radiusBase / radiusWheel);

    // Sending position to 3Mxl
    leftWheelMotor->setPosition(position + posleft + wheelDifference);  // setting left wheel postion
    rightWheelMotor->setPosition(position + posright - wheelDifference);//setting right wheel position
}

void mobile_base::Base::speed(const geometry_msgs::Twist::ConstPtr& msg){
    
    // Reading velocity from topic
    float velocity = msg->linear.x;
    float angle = msg->angular.z;
    
    // Calculating velocity for each motor
    float linearLeftSpeed, linearRightSpeed, angularLeftSpeed, angularRightSpeed;
    linearLeftSpeed = linearRightSpeed = velocity / radiusWheel;
    angularLeftSpeed = angularRightSpeed = angle * (radiusBase / radiusWheel);
    
    // Sending speeds to 3mxl
    leftWheelMotor->setVelocity(linearLeftSpeed + angularLeftSpeed); // setting left wheel postion
    rightWheelMotor->setVelocity(linearRightSpeed - angularRightSpeed);//setting right wheel position
    
}


void mobile_base::Base::posspeed(const geometry_msgs::Twist::ConstPtr& msg) {
  
  // Base is nonholonomic, warn if sent a command we can't execute
  if (msg->linear.y || msg->linear.z || msg->angular.x || msg->angular.y) {
    ROS_WARN("Cannot use given position linear: [%d, %d, %d], angular: [%d, %d, %d]", msg->linear.x, msg->linear.y, msg->linear.z, msg->angular.x, msg->angular.y, msg->angular.z);
    return;
  }
  
  leftWheelMotor->motor->reset();
  rightWheelMotor->motor->reset();

}

void mobile_base::Base::spin(){

    ros::Rate rate(2); // rate at which position published (hertz)
    aidu_mobile_base::State state;
    int countdown = 15;
    while(ros::ok()) {
      
	// Getting position
	leftWheelMotor->motor->getPosAndSpeed();
        state.leftpos = leftWheelMotor->motor->presentPos();
	state.leftspeed = leftWheelMotor->motor->presentSpeed();
	
	rightWheelMotor->motor->getPosAndSpeed();
        state.rightpos = rightWheelMotor->motor->presentPos();
	state.rightspeed = rightWheelMotor->motor->presentSpeed();
	
	// Convert the wheel positions to meters
	//state.leftpos = state.leftpos * radiusWheel;
	//state.rightpos = state.rightpos * radiusWheel;
	
	// Compute angle
	state.angle = ((state.leftpos * radiusWheel / radiusBase)-(state.rightpos * radiusWheel / radiusBase)) / 2.0;
	
	// Only publish state when the motors have been recently active
	if(state.rightspeed == 0 && state.leftspeed == 0) {
	  if(countdown > 0) {
	    countdown--;
	  }
	} else {
	  countdown = 15;
	}
	if(countdown > 0) {
	  pospublisher.publish(state);
	}
	
	// Spin ros
        ros::spinOnce();
	rate.sleep();

    }
}

mobile_base::Base::~Base() {
  delete leftWheelMotor;
  delete rightWheelMotor;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "base");
  mobile_base::Base base;
  base.spin();
  return 0;
}
