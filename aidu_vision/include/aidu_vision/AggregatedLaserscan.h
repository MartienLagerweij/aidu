#ifndef AIDU_VISION__AGGREGATEDLASERSCAN_H
#define AIDU_VISION__AGGREGATEDLASERSCAN_H

#include <ros/ros.h>
#include <aidu_core/node.h>
#include <sensor_msgs/LaserScan.h>
#include <aidu_vision/DistanceSensors.h>


namespace aidu {
    namespace vision {
        class AggregatedLaserscan : public aidu::core::Node {
            public:
                AggregatedLaserscan();
                void laserscanCallback(const sensor_msgs::LaserScan::ConstPtr& scanmsg);///< handles the laserscan input
                void distanceCallback(const aidu_vision::DistanceSensors::ConstPtr& distmsg);///< handles the sensor input
            protected:
                ros::Subscriber laserscanSubscriber, distanceSubscriber;
                ros::Publisher scanPublisher;
        };
    }
}


#endif