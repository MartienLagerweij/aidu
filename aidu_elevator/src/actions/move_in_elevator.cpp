#include <ros/ros.h>
#include <aidu_elevator/actions/move_in_elevator.h>
#include <sensor_msgs/LaserScan.h>
#include <aidu_robotarm/robot_arm_positions.h>
#include <math.h>

using namespace aidu::elevator;

MoveInElevator::MoveInElevator(ros::NodeHandle* nh) : Action::Action(nh) {
    
    // Set up subscribers
    laserscanSubsciber = nh->subscribe("/scan", 1, &MoveInElevator::laserscancallback, this);
    sensor_sub = nh->subscribe("/sensors", 1, &MoveInElevator::sensorcallback, this);
    // Set up publishers
    basepositionPublisher= nh->advertise<geometry_msgs::Twist>("/pos",1);
    
    // Initialize variables
    rotationdirection=0.0;
    

    
}

MoveInElevator::~MoveInElevator() {
}

void MoveInElevator::execute() {
  geometry_msgs::Twist position;
  position.linear.x=0.5;
  position.linear.z=0.;
  basepositionPublisher.publish(position);
  ros::spinOnce();
  sleep(2);
  
  if (rotationdirection==1){
    position.linear.x=0.0;
    position.linear.z=0.7825;
    basepositionPublisher.publish(position);
    ros::spinOnce();
    sleep(2);
    position.linear.x=-0.2;
    position.linear.z=0.0;
    basepositionPublisher.publish(position);
    ros::spinOnce();
    sleep(2);
    position.linear.x=0.0;
    position.linear.z=-4.7165;
    basepositionPublisher.publish(position);
    ros::spinOnce();
    sleep(2);
  }
  else if (rotationdirection==-1){
    
  }
  
      
}

bool MoveInElevator::finished() {
 return false;
}


void MoveInElevator::sensorcallback(const aidu_vision::DistanceSensors::ConstPtr& dist_msg){
    front_left=dist_msg->Frontleft; 
    front_right=dist_msg->Frontright;
    dist_arm=dist_msg->arm;
}


void MoveInElevator::laserscancallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg){
  int size =(scan_msg->angle_max-scan_msg->angle_min)/(scan_msg->angle_increment);
  double dist_treshold=1.5; //distance in meters
  float min_x=INFINITY;
  float max_x=-INFINITY;
  float* x = new float[size];
  float* y = new float[size];
  double angle =scan_msg->angle_min;
  for (int i=0;i<size;i++){
    x[i]=sin(angle)*scan_msg->ranges[i];
    y[i]=cos(angle)*scan_msg->ranges[i];
    angle+=scan_msg->angle_increment;
    if (y[i] > dist_treshold){
      min_x=std::min(min_x,x[i]);
      max_x=std::max(max_x,x[i]);
    }    
  }
  if (fabs(min_x) > max_x){
    rotationdirection=1;
  } else {
    rotationdirection=-1;
  }
  
  delete x;
  delete y;
}






