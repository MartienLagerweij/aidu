
#include <aidu_mobile_base/base.h>
#include <aidu_mobile_base/Pos.h>
#include <geometry_msgs/Twist.h>
#include <aidu_mobile_base/State.h>
#include <threemxl/dxlassert.h>
#include <math.h>
#include <algorithm>

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
    posspeedsubscriber = nh->subscribe("posspeed", 1, &mobile_base::Base::posspeed, this);
    speedpublisher = nh->advertise<geometry_msgs::Twist>("speed", 1);
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
    
    // Compute position-based wheel position
    position = position / radiusWheel;
    
    // Compute angle-based wheel position
    float wheelDifference = angle * (radiusBase / radiusWheel);

    // Sending position to 3Mxl
    leftWheelMotor->setPosition(position + wheelDifference); // setting left wheel postion
    rightWheelMotor->setPosition(position - wheelDifference); //setting right wheel position
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
    ROS_WARN("Cannot use given position: linear:[%f, %f, %f], angular:[%f, %f, %f]", msg->linear.x, msg->linear.y, msg->linear.z, msg->angular.x, msg->angular.y, msg->angular.z);
    return;
  }
  
  // Resets the positions so we count from 0
  this->resetPos();
  
  // Get positions, targets and errors as polar coordinates from base origin
  double actualPos = (getLeftPos() + getRightPos()) / 2.0;
  double actualAngle = getAngle();
  double targetPos = msg->linear.x;
  double targetAngle = msg->angular.z;
  double errorPos = targetPos - actualPos;
  double errorAngle = targetAngle - actualAngle;
  double prevErrorAngle = errorAngle;
  
  // Initialize values for our custom PD controller
  double epsilon = 0.01;
  double KpL = 1.0;
  double KpA = 1.8;
  double KdA = 32.0;
  double maxAngularSpeed = 1.57;
  double maxLinearSpeed = 0.3;
  
  // The message that will be sent
  geometry_msgs::Twist twist;
  
  // PD controller loop. This will exit when the error is smaller than epsilon
  ros::Rate loop(10);
  while(ros::ok() && (fabs(errorPos) > epsilon || fabs(errorAngle) > epsilon)) {
    
    // Get current positions
    actualPos = (getLeftPos() + getRightPos()) / 2.0;
    actualAngle = getAngle();
    
    // Calculate errors
    errorPos = targetPos - actualPos;
    errorAngle = targetAngle - actualAngle;
    
    // Compute derivative of angle error
    double derivative = (errorAngle - prevErrorAngle) / 10;
    prevErrorAngle = errorAngle;
    
    // Set twist message values, bounded by maximum speeds
    twist.linear.x = std::max(-maxLinearSpeed, std::min(maxLinearSpeed, errorPos * KpL));
    twist.angular.z = std::max(-maxAngularSpeed, std::min(maxAngularSpeed, errorAngle * KpA + derivative * KdA ));
    
    // Publish twist message
    speedpublisher.publish(twist);
    
    // Loop
    ros::spinOnce();
    loop.sleep();
  }
  
  // Ensure 0 speed at the end
  twist.linear.x = 0;
  twist.angular.z = 0;
  speedpublisher.publish(twist);
  ros::spinOnce();
  
}

double mobile_base::Base::getAngle() {
  return ((getLeftPos() / radiusBase)-(getRightPos() / radiusBase)) / 2.0;
}

double mobile_base::Base::getLeftPos() {
  leftWheelMotor->motor->getLinearPos();
  return leftWheelMotor->motor->presentLinearPos() - initialLeftPos;
}

double mobile_base::Base::getRightPos() {
  rightWheelMotor->motor->getLinearPos();
  return rightWheelMotor->motor->presentLinearPos() - initialRightPos;
}

void mobile_base::Base::resetPos() {
  leftWheelMotor->motor->getLinearPos();
  rightWheelMotor->motor->getLinearPos();
  initialLeftPos = leftWheelMotor->motor->presentLinearPos();
  initialRightPos = rightWheelMotor->motor->presentLinearPos();
}

void mobile_base::Base::spin(){

    ros::Rate rate(120); // rate at which position published (hertz)
    aidu_mobile_base::State state;
    int countdown = 15;
    while(ros::ok()) {
      
	// Getting position, speed and angle
	leftWheelMotor->motor->getPosAndSpeed();
        state.leftpos = getLeftPos();
	state.leftspeed = leftWheelMotor->motor->presentSpeed();
	rightWheelMotor->motor->getPosAndSpeed();
        state.rightpos = getRightPos();
	state.rightspeed = rightWheelMotor->motor->presentSpeed();
	state.angle = getAngle();
	
	
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
	} else {
	  resetPos();
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
