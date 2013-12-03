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
    targetdistance=0.90;// (m)
    targetangle=0;
    
  //ROS_INFO("created sensor handler");  
}

 
  
void LaserScanHandler::LaserScanCallback(const sensor_msgs::LaserScan::ConstPtr& scanmsg){
  
  //Position and angle control with Laserscan data
  geometry_msgs::Twist twist;
  double range_min=scanmsg->range_min;
  double range_max=scanmsg->range_max;
  int size =(scanmsg->angle_max-scanmsg->angle_min)/(scanmsg->angle_increment);
  int number=0;
  double total=0,avg_dist=0;;
  
  //calculating distance from wall
  for (int i=0;i<=size;i++){
    if(scanmsg->ranges[i]>=range_min && scanmsg->ranges[i]<=range_max){
      total+=scanmsg->ranges[i];
      number++;
    }
  }
  if(number!=0)
  avg_dist=total/number;
  ROS_INFO("avg dist: %f  number:%d",avg_dist,number);
  // initialising control values
  double KpL = 1.5;
  double KdL=1.0;
  double KpA =1.7;
  double KdA = 10.0;
  //calculating angle with wall
  
  //calculating errors
  double errP;
  errP=avg_dist-targetdistance;
  
  
  //sending speed to Base
  twist.linear.x=errP*KpL;
  
  
  

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