#include <aidu_vision/laserscancombination.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <vector>

using namespace aidu;

LaserScanCombination::LaserScanCombination(): core::Node::Node(){

  //subscribing and pubishing
  scanmsg = nh->subscribe("/scan", 1, &LaserScanCombination::LaserScanCallback, this);
  distmsg= nh->subscribe("/sensors", 1, &LaserScanCombination::DistCallback, this);
  laserscanpublisher= nh->advertise<sensor_msgs::LaserScan>("/scan2",1);
}

  void LaserScanCombination::DistCallback(const aidu_vision::DistanceSensors::ConstPtr& distmsg){
    distLeft=distmsg->Left/1000.0;
    distRight=distmsg->Right/1000.0;
  }
  
  void LaserScanCombination::LaserScanCallback(const sensor_msgs::LaserScan::ConstPtr& scanmsg){
    // Distance sensor position
    const double xSensor=0.3; 	// x distance from kinect
    const double ySensor=0.3;	// y distance from kinect
    const double rMin=sqrt(pow(xSensor,2)+pow(ySensor,2));
    const double thetaMax=asin(ySensor/rMin)+1.5705;
    ROS_INFO("thetamax:%f",thetaMax);
    //getting lasercan data
    sensor_msgs::LaserScan scanmsg2;
    double angle_min=scanmsg->angle_min;
    double angle_max=scanmsg->angle_max;
    double angle_increment=scanmsg->angle_increment;
    int size =(scanmsg->angle_max-scanmsg->angle_min)/(scanmsg->angle_increment);
    ROS_INFO("size:%d", size);
  
    //adding left and right sensor data to laserscan
    int negaddionalPoints=floor((thetaMax+angle_min)/angle_increment);
    int posaddionalPoints=ceil((thetaMax-angle_max)/angle_increment);
    int totalsize=negaddionalPoints+size+posaddionalPoints;
    ROS_INFO("totalsize:%d",totalsize);
    //Copying laser scan data to new array filled with NaNs
    float* ranges = new float[totalsize];
    
    for (int i=0;i<negaddionalPoints;i++){
      ranges[i]=std::numeric_limits<double>::quiet_NaN();
    }
    for (int i=negaddionalPoints;i<(negaddionalPoints+size);i++){
      ranges[i]=scanmsg->ranges[i-negaddionalPoints];
    }
    for (int i=(negaddionalPoints+size);i<(totalsize);i++){
      ranges[i]=std::numeric_limits<double>::quiet_NaN();
    }
    
    //transforming coordinate frame of the ultrasonic sensor data to lasercan frame
    double rLeft=sqrt(pow((xSensor+distLeft),2)+pow(ySensor,2));
    double rRight=sqrt(pow((xSensor+distRight),2)+pow(ySensor,2));
    double thetaLeft=asin(ySensor/rLeft)+1.5705;
    double thetaRight=asin(ySensor/rRight)+1.5705;
    ROS_INFO("thetaleft:%f    thetaright:%f",thetaLeft,thetaRight);
    
    // Adding ultrasonic sensor data in the laserscan array
    int posLeft=floor((thetaMax-thetaLeft)/angle_increment);
    int posRight=totalsize-ceil((thetaMax-thetaRight)/angle_increment);
    ROS_INFO("posleft=%d    posright:%d",posLeft,posRight);
    ranges[posLeft]=distLeft;
    ranges[posRight]=distRight;
    
    ROS_INFO("sizeof:%d", sizeof(ranges));
    std::vector<float> ranges2 (ranges, ranges + totalsize);
  
    //setting new laserscan msg
    scanmsg2.angle_min=-thetaMax;
    scanmsg2.angle_max=-thetaMax+totalsize*angle_increment;
    scanmsg2.ranges=ranges2;
    scanmsg2.angle_increment=scanmsg->angle_increment;
    scanmsg2.range_max=scanmsg->range_max;
    scanmsg2.range_min=scanmsg->range_min;
    scanmsg2.scan_time=scanmsg->scan_time;
    scanmsg2.time_increment=scanmsg->time_increment;
    scanmsg2.header.frame_id=scanmsg->header.frame_id;
    scanmsg2.header.stamp=scanmsg2.header.stamp;
    scanmsg2.intensities=scanmsg->intensities;
    laserscanpublisher.publish(scanmsg2);
    
    
    delete[] ranges;
  }
  
  int main(int argc, char** argv) {
  ros::init(argc, argv, "laserscancombination");
  LaserScanCombination laserscancombination;
  laserscancombination.spin();
  return 0;
}