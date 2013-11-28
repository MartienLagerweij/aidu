#include <aidu_vision/WPass2.h>
#include <geometry_msgs/Twist.h>
#include <math.h>

using namespace aidu;

SensorHandler::SensorHandler(): core::Node::Node(){

  //subscibing and pubishing
  sensormsg = nh->subscribe("/sensors", 1, &SensorHandler::sensorcallback, this);
  speedpublisher = nh->advertise<geometry_msgs::Twist>("/base/speed",1);
  pospublisher= nh->advertise<geometry_msgs::Twist>("/base/pos",1);
  
  // iniatilising values
  prevErrorAngle=0.0;
  targetdistance=1;// (m)
  targetangle=0;
  distance_between_sensors=0.5; // (m)
  middle_sensor_range=0.01; // (m)
  maxAngularSpeed = 1.4;
  maxLinearSpeed=0.3;
    
  ROS_INFO("created sensor handler");  
}

void SensorHandler::sensorcallback(const aidu_vision::DistanceSensors::ConstPtr& sensormsg){
  /*
  // Setting distance from wall 
  geometry_msgs::Twist twist;
  double speed,presentdist,errorPos,distmiddle,distleft,distright;
  distmiddle=sensormsg->Frontmiddle/1000;
  distleft=sensormsg->Frontleft/1000;
  distright=sensormsg->Frontright/1000;
  ROS_INFO("left:%f right:%f  middle:%f",distleft,distright,distmiddle);
  //reading distance from the wall
  presentdist=(distleft+distright)/2; // current distance from wall (m)
  if (presentdist<=middle_sensor_range){ // if infrared sensor in range then use it as well
    presentdist=(presentdist+distmiddle)/2;
  }
  
  //calculting distance to move
  errorPos=(presentdist-targetdistance); //(m)
  twist.linear.x = std::max(-maxLinearSpeed, std::min(maxLinearSpeed, errorPos * KpL));
  //publishing distance message
  speedpublisher.publish(twist);
  ROS_INFO("published linear.x :%f",twist.linear.x); 
  ros::spinOnce();
  */
  
  
  
  //Setting distance and angle
  geometry_msgs::Twist twist;
  
  // initialising
  double KpL = 1.5;
  double KpA =2.0;
  double KdA = 30.0;
  double distleft,distright,distmiddle, diffdist,angle,dist;
  
  //getting sensor data
  distleft=sensormsg->Frontleft/1000.0;
  distright=sensormsg->Frontright/1000.0;
  distmiddle=sensormsg->Frontmiddle/1000.0;
  
  //calculting angle with the wall
  diffdist=distleft-distright;
  angle=atan(diffdist/distance_between_sensors);
  //ROS_INFO("the angle=%f degrees",angle/3.141592*180.0);
  
  
  //calculting distance from wall
  dist=(distleft+distright)/2;
  if (dist<=middle_sensor_range){ // if in range use the infra red sensor as wel
    dist=(dist+distmiddle)/2;
  }
  
  //calculting errors
  double errA,errP;
  errA=angle-targetangle;
  errP=dist-targetdistance;
  
  // Compute derivative of angle error
  double derivative = (errA - prevErrorAngle) / 10;
  prevErrorAngle = errA;
    
  // Set twist message values, bounded by maximum speeds
  twist.linear.x = std::max(-maxLinearSpeed, std::min(maxLinearSpeed, errP * KpL));
  twist.angular.z = std::max(-maxAngularSpeed, std::min(maxAngularSpeed, errA * KpA + derivative * KdA ));
  
  
  //publishing distances
  twist.linear.x=dist-targetdistance;
  twist.angular.z=angle;
  ROS_INFO("speed:%f  angle:%f",twist.linear.x,twist.angular.z);
  speedpublisher.publish(twist);
  ros::spinOnce();
  
  
}












int main(int argc, char** argv) {
  ros::init(argc, argv, "wpass2");
  SensorHandler sensorhandler;
  sensorhandler.spin();
  return 0;
}