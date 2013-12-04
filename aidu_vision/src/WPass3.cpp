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
    targetdistance=0.50;// (m)
    targetangle=0;
    
  //ROS_INFO("created sensor handler");  
}

 
  
void LaserScanHandler::LaserScanCallback(const sensor_msgs::LaserScan::ConstPtr& scanmsg){
  
  //receiving Kinect data
  double range_min=scanmsg->range_min;
  double range_max=scanmsg->range_max;
  double angle_increment=scanmsg->angle_increment;
  int size =(scanmsg->angle_max-scanmsg->angle_min)/(scanmsg->angle_increment);
  
  //transforming to x and y coordinates
  double angle=scanmsg->angle_min;
  double x[size],y[size];
  for (int i=0;i<=size;i++){
    y[i]=cos(angle)*scanmsg->ranges[i];
    x[i]=sin(angle)*scanmsg->ranges[i];
    angle+=angle_increment;
  }
  geometry_msgs::Twist twist;
  
  /*
  //Position and angle control with Laserscan data
  
  int number=0;
  double total=0,avg_dist=0;;
 
  //calculating distance from wall
  avg_dist=average(y,size);
  ROS_INFO("dist %f",avg_dist);
  
  //calculating angle of the wall
  double diffdist[size-1];
  for (int i=0;i<=size-1;i++){
    diffdist[i]=(y[i]-y[i+1])/(x[i]-x[i+1]);
  }
  double avg_diffdist=average(diffdist,size-1);
  double theta=atan(avg_diffdist);
  ROS_INFO("theta:%f",theta);
  
  // initialising control values
  double KpL = 1.5;
  //double KdL=1.0;
  double KpA =1.7;
  //double KdA = 10.0;
  //calculating angle with wall
  
  //calculating errors
  double errP,errA;
  errP=avg_dist-targetdistance;
  errA=theta-targetangle;
  
  //sending speed to Base
  twist.linear.x=errP*KpL;
  twist.angular.z=errA*KpA;
   
  // Basis laten stoppen als muur te dichtbij
  double treshhold=0.1;
  for (int i=0;i<=size;i++){
    if(y[i]<treshhold){
      twist.linear.x=0.0;
      twist.angular.z=0.0;
    }
  }
  
  */
  
  // Door de deur reiden
  double baseWidth=0.8; // breedte van basis + marge 
  double treshhold=1.0; // (m)
  double kp=1.7;
  double angleCorrectionleft=0.0,angleCorrectionright=0.0;
  for (int i=0; i<=size; i++){
    //ROS_INFO("x[%d]=%f  y[%d]=%f",i,x[i],i,y[i]);
    if (x[i]>-baseWidth/2.0 && x[i]<-0.1 && y[i]<treshhold){
      angleCorrectionleft=std::min((y[i]-treshhold)*kp,angleCorrectionleft);
    }
    if (x[i]<baseWidth/2.0 && x[i]>0.1 && y[i]<treshhold){
      angleCorrectionright=std::max((y[i]-treshhold)*-kp,angleCorrectionright);
    }
  }
  
  
  twist.angular.z=angleCorrectionleft+angleCorrectionright;
  twist.linear.x=0.4;
  ROS_INFO("speed:%f  angle:%f",twist.linear.x,twist.angular.z);
  speedpublisher.publish(twist);
  ros::spinOnce();
  
}





double LaserScanHandler::average(double array[], int size){
  double total=0;
  int number=0;
  for (int i=0;i<=size;i++){
    if(isnan(array[i])==0){
      total+=array[i];
      number++;
    }
  }
  if(number!=0){
    return total/(double)number;}
  else{
      return(0);
  }
 
}







int main(int argc, char** argv) {
  ros::init(argc, argv, "wpass3");
  LaserScanHandler laserscanhandler;
  laserscanhandler.spin();
  return 0;
}