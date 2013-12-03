#include <aidu_vision/WPass3.h>
#include <geometry_msgs/Twist.h>
#include <math.h>

using namespace aidu;

LaserScanHandler::LaserScanHandler(): core::Node::Node(){

  //subscribing and pubishing
  scanmsg = nh->subscribe("/scan", 1, &LaserScanHandler::LaserScanCallback, this);
  speedpublisher = nh->advertise<geometry_msgs::Twist>("/base/speed",1);
  
  // iniatilising values
  
    //control values
    targetdistance=0.40;// (m)
    targetangle=0;
    
  //ROS_INFO("created sensor handler");  
}

 
  
void LaserScanHandler::LaserScanCallback(const sensor_msgs::LaserScan::ConstPtr& scanmsg){
  
  //Position and angle control with Laserscan data
  geometry_msgs::Twist twist;
  
  // initialising control values
  double KpL = 1.5;
  double KdL=1.0;
  double KpA =1.7;
  double KdA = 10.0;
  
  //calculating distance from wall
  
  
  //calculating angle with wall 
  
  
  

  ROS_INFO("speed:%f  angle:%f",twist.linear.x,twist.angular.z);
  speedpublisher.publish(twist);
  ros::spinOnce();
  
}












int main(int argc, char** argv) {
  ros::init(argc, argv, "wpass3");
  LaserScanHandler laserscanhandler;
  laserscanhandler.spin();
  return 0;
}