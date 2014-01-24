#include <ros/ros.h>
#include <aidu_elevator/actions/movetobutton.h>
#include <geometry_msgs/Twist.h>
#include <aidu_vision/DistanceSensors.h>
#include <geometry_msgs/Twist.h>

#DEFINE HORIZONTAL_FOV 1.07115918
#DEFINE VERTICAL_FOV 0.644356361
#DEFINE SIGN(X) (X > 0 ? 1 : (X < 0 ? -1 : 0))

using namespace aidu::elevator;

double convert(double fov, double img_x, double z, double resolution) {
    double theta = -fov/2.0+(fov*img_x/resolution);
    double x = tan(theta)*z;
    return(x);
}


MoveToButton::MoveToButton(ros::NodeHandle* nh, int button) : Action::Action(nh) {
    //publisher
    positionPublisher = nh->advertise<geometry_msgs::Twist>("/pos",1,true);
    //subscriber
    sensorSubscriber = nh->subscribe("/sensors", 1, &MoveToButton::sensorCallback, this);
    buttonSubscriber = nh->subscribe("/elevator/button/classified", 1, &MoveToButton::buttonCallback, this);
    distance = -1.0;
    buttonX = -1000.0;
    this->button = button;
}

MoveToButton::~MoveToButton() {
    
}

void MoveToButton::execute() {
  bool door_open=false;
  ROS_INFO("Executing action: move to button");
  ros::Rate loop(20);
  
  // Wait for distance
  ROS_INFO("Waiting for distance measurement");
  while(ros::ok() && distance == -1.0) {
      ros::spinOnce();
      loop.sleep();
  }
  
  // Wait for button
  ROS_INFO("Waiting for button");
  while(ros::ok() && buttonX == -1000.0) {
      ros::spinOnce();
      loop.sleep();
  }
  
  // Move
  ROS_INFO("Moving base");
  sleep(1);
  this->moveBase(0.0, SIGN(buttonX) * 1.57);
  sleep(3);
  this->moveBase(fabs(buttonX), 0.0);
  sleep(4);
  this->moveBase(0.0,  -SIGN(buttonX) * 1.57);
  sleep(3);
  this->moveBase(distance - 0.10, 0.0);
  this->inFrontOfButton = true;
  ROS_INFO("door open");
}

void MoveToButton::moveBase(double linear, double angular) {
    geometry_msgs::Twist position;
    position.angular.z = angular;
    position.linear.x = linear;
    positionPublisher.publish(position);
}

bool MoveToButton::finished() {
  return inFrontOfButton;
}

void MoveToButton::sensorCallback(const aidu_vision::DistanceSensors::ConstPtr& dist_msg) {
    distance = (dist_msg->Frontleft + dist_msg->Frontright) / 2.0;
    //ROS_INFO("distance:%f",distance);
}

void MoveToButton::buttonCallback(const aidu_elevator::Button::ConstPtr& message) {
    if (message->button_type == button) {
        buttonX = convert(HORIZONTAL_FOV, message->x, distance, 1280);
    }
}

