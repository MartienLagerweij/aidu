
#include <aidu_mobile_base/base.h>
#include <aidu_mobile_base/Pos.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Transform.h>
#include <aidu_mobile_base/BaseState.h>
#include <threemxl/dxlassert.h>
#include <math.h>
#include <algorithm>

#define BOUND(x,y) std::max(std::min(x, y), -y)

using namespace aidu;

mobile_base::Base::Base() : core::Node::Node() {
  
    // Read parameters from config file
    std::string motor_port_name, motor_config_name;
    nh->getParam("motor_port", motor_port_name);
    nh->getParam("motor_config", motor_config_name);
    
    // Get sizes from launch parameters
    nh->getParam("wheel_base", radiusBase);
    radiusBase *= 0.5;
    nh->getParam("wheel_diameter", radiusWheel);
    radiusWheel *= 0.5;
    
    //creating left and right motor
    leftWheelMotor = new mobile_base::Motor("left", motor_port_name, motor_config_name, radiusWheel);
    rightWheelMotor = new mobile_base::Motor("right", motor_port_name, motor_config_name, radiusWheel);

    // Subscribing
    positionSubscriber = nh->subscribe("pos", 1, &mobile_base::Base::position, this);
    speedSubscriber = nh->subscribe("/cmd_vel", 1, &mobile_base::Base::speed, this);
    //configSubscriber = nh->subscribe("config", 1, &mobile_base::Base::setConfig, this);
    leftWheelSubscriber = nh->subscribe("/lwheel_vtarget", 1, &mobile_base::Base::leftWheelSpeed, this);
    rightWheelSubscriber = nh->subscribe("/rwheel_vtarget", 1, &mobile_base::Base::rightWheelSpeed, this);
    
    // Publishing 
    statePublisher = nh->advertise<aidu_mobile_base::BaseState>("state",1);
    maximumLinearVelocity = 10;
    maximumAngularVelocity = 10;
  
}

void mobile_base::Base::leftWheelSpeed(const std_msgs::Float32::ConstPtr& msg) {
    leftWheelMotor->setVelocity(msg->data / radiusWheel);
}

void mobile_base::Base::rightWheelSpeed(const std_msgs::Float32::ConstPtr& msg) {
    rightWheelMotor->setVelocity(msg->data / radiusWheel);
}

void mobile_base::Base::position(const geometry_msgs::Twist::ConstPtr& msg) {
  
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
    float velocity = BOUND(msg->linear.x, maximumLinearVelocity);
    float angle = BOUND(msg->angular.z, maximumAngularVelocity);
    
    // Calculating velocity for each motor
    float linearLeftSpeed, linearRightSpeed, angularLeftSpeed, angularRightSpeed;
    linearLeftSpeed = linearRightSpeed = velocity / radiusWheel;
    angularLeftSpeed = angularRightSpeed = angle * (radiusBase / radiusWheel);
    
    // Sending speeds to 3mxl
    leftWheelMotor->setVelocity(linearLeftSpeed - angularLeftSpeed); // setting left wheel postion
    rightWheelMotor->setVelocity(linearRightSpeed + angularRightSpeed);//setting right wheel position
    
}

/*void mobile_base::Base::setConfig(const aidu_mobile_base::Config::ConstPtr& msg) {
    this->maximumLinearVelocity = msg->maximumLinearSpeed;
    this->maximumAngularVelocity = msg->maximumAngularSpeed;
    if (this->getLinearVelocity() > maximumLinearVelocity || this->getAngularVelocity() > maximumAngularVelocity) {
        geometry_msgs::Twist twist;
        twist.linear.x = BOUND(getLinearVelocity(), maximumLinearVelocity);
        twist.angular.z = BOUND(getAngularVelocity(), maximumAngularVelocity);
        speedPublisher.publish(twist);
    }
}*/

double mobile_base::Base::getAngle() {
    return ((rightWheelMotor->getLinearPosition() - leftWheelMotor->getLinearPosition()) / 2.0) / radiusBase;
}

double mobile_base::Base::getLinearVelocity() {
    return ((leftWheelMotor->getLinearVelocity() + rightWheelMotor->getLinearVelocity()) / 2.0);
}

double mobile_base::Base::getAngularVelocity() {
    return ((rightWheelMotor->getLinearVelocity() - leftWheelMotor->getLinearVelocity()) / 2.0) / radiusBase;
}

void mobile_base::Base::resetPos() {
    leftWheelMotor->reset();
    rightWheelMotor->reset();
}

void mobile_base::Base::publishState() {
    
    // Initialize
    aidu_mobile_base::BaseState state;
    
    // Set positions and speeds
    state.left.position = leftWheelMotor->getLinearPosition();
    state.left.speed = leftWheelMotor->getLinearVelocity();
    state.right.position = rightWheelMotor->getLinearPosition();
    state.right.speed = rightWheelMotor->getLinearVelocity();
    state.angle.position = getAngle();
    state.angle.speed = getAngularVelocity();
    
    // Publish the base state
    statePublisher.publish(state);
    
}

void mobile_base::Base::spin(){

    ros::Rate rate(30); // rate at which position published (hertz)
    resetPos(); // Reset the position initially
    
    while(ros::ok()) {
        
        // Update motor information
        leftWheelMotor->update();
        rightWheelMotor->update();
      
        // Publish information
        publishState();
        
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
