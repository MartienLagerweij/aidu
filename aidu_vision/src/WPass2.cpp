#include <aidu_vision/WPass2.h>
#include <geometry_msgs/Twist.h>
#include <math.h>

using namespace aidu;

SensorHandler::SensorHandler(): core::Node::Node(){

  //subscribing and pubishing
  sensormsg = nh->subscribe("/sensors", 1, &SensorHandler::sensorcallback, this);
  speedpublisher = nh->advertise<geometry_msgs::Twist>("/base/speed",1);
  pospublisher= nh->advertise<geometry_msgs::Twist>("/base/pos",1);
  statesubscriber= nh->subscribe("/base/state", 1, &SensorHandler::State, this);
  
  // iniatilising values
  
    //memory values
    prevErrorAngle=0.0;
    prevErrorPos=0.0;
    i=0;
    correctionFactor=2.5;
    //control values
    targetdistance=0.30;// (m)
    targetangle=0;
    distance_between_sensors=0.37; // (m)
    maxAngularSpeed =5.0;
    maxLinearSpeed=2.0;
    
  //ROS_INFO("created sensor handler");  
}

void SensorHandler::State(const aidu_mobile_base::State::ConstPtr& statemsg){
 // motorstate.leftpos=statemsg->leftpos;
 // motorstate.rightpos=statemsg->rightpos;
 // motorstate.angle=statemsg->angle;
}
  
  
  
  
void SensorHandler::sensorcallback(const aidu_vision::DistanceSensors::ConstPtr& sensormsg){
  // Three nodes: (only one should be uncommented)
  //		  first only position control with sensor data
  //		  second position and angle control with sensor data
  //		  third position and angle feedback with sensor data and odometry
  
  /*
  // Position control with sensor data 
  geometry_msgs::Twist twist;
  double KpL=1.0;
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
  
  
  
  //Position and angle control with sensor data
  geometry_msgs::Twist twist;
  
  // initialising
  double KpL = 1.2;
  double KdL=1.0;
  double KpA =1.7;
  double KdA = 10.0;
  double epsilon=0.01;
  double distleft,distright, diffdist,angle,dist;
  double loopRate=20;
  
  //getting sensor data
  distleft=std::min(sensormsg->Frontleft/1000.0 , 3.0);
  distright=std::min(sensormsg->Frontright/1000.0 , 3.0);
  
  //initialising and computing avarage sensor data
    if (first){
    first=false;
    prevDistLeft[0]=prevDistLeft[1]=prevDistLeft[2]=prevDistLeft[3]=distleft;
    prevDistRight[0]=prevDistRight[1]=prevDistRight[2]=prevDistRight[3]=distright;
  }
  double avgDistLeft=(prevDistLeft[0]+prevDistLeft[1]+prevDistLeft[2]+prevDistLeft[3])/4.0;
  double avgDistRight=(prevDistRight[0]+prevDistRight[1]+prevDistRight[2]+prevDistRight[3])/4.0;
  
  //Remembering sensor data
  prevDistLeft[i]=distleft;
  prevDistRight[i]=distright;
  i++;
  if (i>=4)i=0;
  
  if (distleft>(correctionFactor*avgDistLeft)){
    distleft=avgDistLeft-avgDistLeft*0.2;
  }
    if (distright>(correctionFactor*avgDistRight)){
    distright=avgDistRight-avgDistRight*0.2;
    }
  ROS_INFO(" avg left:%f  avg right:%f",avgDistLeft,avgDistRight);

  
  //calculting angle with the wall
  diffdist=distleft-distright;
  angle=atan(diffdist/distance_between_sensors);
  //ROS_INFO("the angle=%f degrees",angle/3.141592*180.0);
  
  
  //calculting distance from wall
  dist=(distleft+distright)/2;

  //calculting errors
  double errA,errP;
  errA=angle-targetangle;
  errP=dist-targetdistance;
  
  // Compute derivative of angle error
  double derivative = (errA - prevErrorAngle) /loopRate;
  prevErrorAngle = errA;
  
  //Compute derivative of pos error
  //double derivativePos=(errP-prevErrorPos)/loopRate;
  //prevErrorPos=errP;
    
  // Set twist message values, bounded by maximum speeds
  if (fabs(errP)>epsilon){
    twist.linear.x = std::max(-maxLinearSpeed, std::min(maxLinearSpeed, errP * KpL));
  }
  if (dist<=2.5*targetdistance && fabs(errA)>epsilon){
    twist.angular.z = std::max(-maxAngularSpeed, std::min(maxAngularSpeed, errA * KpA + derivative * KdA ));
  } else{
    twist.angular.z=0.0;
  }
  
  ROS_INFO("speed:%f  angle:%f",twist.linear.x,twist.angular.z);
  speedpublisher.publish(twist);
  ros::spinOnce();
  
  
  
  /*
  //Position and angle control with sensor and odometry
  geometry_msgs::Twist twist;
  
  // initialising
  double KpL = 1.5;
  double KpA =0.5;
  double KdA = 30.0;
  double epsilon=0.01;
  double distSensorLeft,distSensorRight,distSensorMiddle, diffdist,angleSensors,distSensors;
  double leftPos,rightPos;
  
  //getting sensor data
  distSensorLeft=sensormsg->Frontleft/1000.0;
  distSensorRight=sensormsg->Frontright/1000.0;
  distSensorMiddle=sensormsg->Frontmiddle/1000.0;
  
  //calculting angle with the wall with sensors
  diffdist=distSensorLeft-distSensorRight;
  angleSensors=atan(diffdist/distance_between_sensors);
  //ROS_INFO("the angle=%f degrees",angle/3.141592*180.0);
  
  //calculting distance from wall with sensors
  distSensors=(distSensorLeft+distSensorRight)/2;
  if (distSensors<=middle_sensor_range){ // if in range use the infra red sensor as wel
    distSensors=(distSensors+distSensorMiddle)/2;
  }
  
  //getting position from 3mxl
  
  
  
  //calculting errors from sensors
  double errA,errP;
  errA=angleSensors-targetangle;
  errP=distSensors-targetdistance;
  
  // Compute derivative of angle error
  double derivative = (errA - prevErrorAngle) / 10;
  prevErrorAngle = errA;
  
  // Set twist message values, bounded by maximum speeds
  if (errP>epsilon)
  twist.linear.x = std::max(-maxLinearSpeed, std::min(maxLinearSpeed, errP * KpL));
  if (distSensors<=1.5*targetdistance && errA>epsilon){
    twist.angular.z = std::max(-maxAngularSpeed, std::min(maxAngularSpeed, errA * KpA + derivative * KdA ));
  } else{
    twist.angular.z=0.0;
  }
  //ROS_INFO("speed:%f  angle:%f",twist.linear.x,twist.angular.z);
  speedpublisher.publish(twist);
  ros::spinOnce();
 */ 
}












int main(int argc, char** argv) {
  ros::init(argc, argv, "wpass2");
  SensorHandler sensorhandler;
  sensorhandler.spin();
  return 0;
}