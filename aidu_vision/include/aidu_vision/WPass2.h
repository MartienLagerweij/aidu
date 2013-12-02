#ifndef AIDU_VISION__WPASS2_H
#define AIDU_VISION__WPASS2_H

#include <ros/ros.h>
#include <aidu_core/node.h>
#include <aidu_vision/DistanceSensors.h>
#include <aidu_mobile_base/State.h>


namespace aidu {
  
  
  class SensorHandler : public aidu::core::Node {
  public:
    SensorHandler();
    void sensorcallback(const aidu_vision::DistanceSensors::ConstPtr& sensormsg);///< handles the sensor input
    void State(const aidu_mobile_base::State::ConstPtr& statemsg);
    
  protected:
    ros::Subscriber sensormsg,statesubscriber;
    ros::Publisher speedpublisher;
    ros::Publisher pospublisher;
    double targetdistance;
    double distance_between_sensors;
    double targetangle,maxAngularSpeed,maxLinearSpeed; ///< distance from wall wanted (mm)
    double prevDistLeft[4],prevDistRight[4], correctionFactor,prevErrorAngle,prevErrorPos; ///< data memory values
    int i; 
    bool first; // true if first measuremment
  };
}




#endif
