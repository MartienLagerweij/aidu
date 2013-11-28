#include <aidu_vision/WPass2.h>
#include <geometry_msgs/Twist.h>
#include <math.h>

using namespace aidu;

SensorHandler::SensorHandler(): core::Node::Node(){

  //subscibing and pubishing
  sensormsg = nh->subscribe("sensors", 1, &SensorHandler::sensorcallback, this);
  speedpublisher = nh->advertise<geometry_msgs::Twist>("/base/speed",1);
  pospublisher= nh->advertise<geometry_msgs::Twist>("/base/pos",1);
  
  // iniatilising values
  targetdistance=1;// (m)
  distance_between_sensors=0.5; // (m)
  middle_sensor_range=0.8; // (m)
    
}

void SensorHandler::sensorcallback(const aidu_vision::DistanceSensors::ConstPtr& sensormsg){
  ROS_INFO("received sensor data");
  double maxLinearSpeed=0.3,KpL=1.0;
  
  // Setting distance from wall 
  geometry_msgs::Twist twist;
  double speed,presentdist,errorPos,distmiddle;
  distmiddle=sensormsg->Frontmiddle;
  
  //reading distance from the wall
  presentdist=(sensormsg->Frontleft+sensormsg->Frontright)/2000; // current distance from wall (m)
  if (presentdist<=middle_sensor_range){ // if infrared sensor in range then use it as well
    presentdist=(presentdist+distmiddle)/2;
  }
  
  //calculting distance to move
  errorPos=(presentdist-targetdistance); //(m)
  twist.linear.x = std::max(-maxLinearSpeed, std::min(maxLinearSpeed, errorPos * KpL));
  //publishing distance message
  speedpublisher.publish(twist);
  ROS_INFO("published linear.x :%f",dist); 
  ros::spinOnce();
  
  
  
  /*
  //Setting distance and angle
  geometry_msgs::Twist twist;
  double distleft,distright,distmiddle, diffdist,angle,dist;
  distleft=sensormsg->Frontleft/1000.0;
  distright=sensormsg->Frontright/1000.0;
  distmiddle=sensormsg->Frontmiddle/1000.0;
  
  //calculting angle with the wall
  diffdist=distleft-distright;
  angle=atan(diffdist/distance_between_sensors);
  ROS_INFO("the angle=%f degrees",angle/3.141592*180.0);
  
  //calculting distance from wall
  dist=(distleft+distright)/2;
  if (dist<=middle_sensor_range){ // if in range use the infra red sensor as wel
    dist=(dist+distmiddle)/2;
  }
  
  //publishing distances
  twist.linear.x=dist-targetdistance;
  twist.angular.z=angle;
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