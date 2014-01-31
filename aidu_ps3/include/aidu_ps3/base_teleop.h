#ifndef AIDU_PS3__BASE_TELEOP_H
#define AIDU_PS3__BASE_TELEOP_H


#include <ros/ros.h>
#include <aidu_core/node.h>
#include <sensor_msgs/Joy.h>


namespace aidu {
  namespace ps3 {
    class TeleopBase : public aidu::core::Node {

    public:
      TeleopBase();
      void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
      
    private:
    
      int linear_, angular_;
      double l_scale_, a_scale_;
      ros::Publisher basepublisher,elevatorpublisher,robotarmpublisher, robotarmspeedpub;
      ros::Subscriber joy_sub_;
      float maxspeed,maxangle;
      bool active;
      
    };
  }
}

#endif