#ifndef AIDU_VISION__WPASS2_H
#define AIDU_VISION__WPASS2_H

#include <ros/ros.h>
#include <aidu_core/node.h>
#include <aidu_vision/DistanceSensors.h>


namespace aidu {
  
  class SensorHandler : public aidu::core::Node {
  public:
    SensorHandler();
    void sensorcallback(const aidu_vision::DistanceSensors::ConstPtr& sensormsg);///< handles the sensor input
    
    
  protected:
    ros::Subscriber sensormsg;
    ros::Publisher speedpublisher;
    ros::Publisher pospublisher;
    double targetdistance,distance_between_sensors,middle_sensor_range; ///< distance from wall wanted (mm)
    
    
    
  };
}




#endif
