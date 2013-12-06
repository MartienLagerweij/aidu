#ifndef AIDU_VISION__LASERSCANCOMBINATION_H
#define AIDU_VISION__LASERSCANCOMBINATION_H

#include <ros/ros.h>
#include <aidu_core/node.h>
#include <aidu_vision/DistanceSensors.h>
#include <sensor_msgs/LaserScan.h>


namespace aidu {
  
  
  class LaserScanCombination : public aidu::core::Node {
  public:
    LaserScanCombination();
    void LaserScanCallback(const sensor_msgs::LaserScan::ConstPtr& scanmsg);///< handles the sensor input
    void DistCallback(const aidu_vision::DistanceSensors::ConstPtr& distmsg);///< handles the sensor input
  protected:
    ros::Subscriber scanmsg,distmsg;
    ros::Publisher laserscanpublisher;
    ros::Time angularTime;
    double distLeft,distRight;
  };
}




#endif
