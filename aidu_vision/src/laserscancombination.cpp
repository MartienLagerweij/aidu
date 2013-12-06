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
    distLeft=distmsg->Left;
    distRight=distmsg->Right;
  }
  
  void LaserScanCombination::LaserScanCallback(const sensor_msgs::LaserScan::ConstPtr& scanmsg){
    
    //getting lasercan data
    sensor_msgs::LaserScan scanmsg2;
    double angle_min=scanmsg->angle_min;
    double angle_max=scanmsg->angle_max;
    double angle_increment=scanmsg->angle_increment;
    int size =(scanmsg->angle_max-scanmsg->angle_min)/(scanmsg->angle_increment);
  
    //adding left and right sensor data to laserscan
    int negaddionalPoints=ceil((1.5705+angle_min)/angle_increment);
    int posaddionalPoints=ceil((1.5705-angle_max)/angle_increment);
    ROS_INFO("negaddi:%d    posaddi:%d  total:%d",negaddionalPoints,posaddionalPoints,(negaddionalPoints+posaddionalPoints+size));
    float ranges[1985];
    
    //std::vector<float> ranges3;
    
    ranges[0]=distRight;
    for (int i=1;i<negaddionalPoints;i++){
      //ranges3.assign(i, std::numeric_limits<float>::quiet_NaN());
      ranges[i]=std::numeric_limits<double>::quiet_NaN();
    }
    for (int i=negaddionalPoints;i<(negaddionalPoints+size);i++){
      //ranges3.assign(i, scanmsg->ranges[i]);
      ranges[i]=scanmsg->ranges[i-negaddionalPoints];
    }
    for (int i=(negaddionalPoints+size);i<(negaddionalPoints+size+posaddionalPoints-1);i++){
      ranges[i]=std::numeric_limits<double>::quiet_NaN();
    }
    ranges[(negaddionalPoints+size+posaddionalPoints-1)]=distLeft;
    
    
    std::vector<float> ranges2 (ranges, ranges + sizeof(ranges) / sizeof(float));
  
  
    //setting new laserscan msg
    scanmsg2.angle_min=-1.5705;
    scanmsg2.angle_max=-1.5705+1985*angle_increment;
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
  }
  
  int main(int argc, char** argv) {
  ros::init(argc, argv, "laserscancombination");
  LaserScanCombination laserscancombination;
  laserscancombination.spin();
  return 0;
}