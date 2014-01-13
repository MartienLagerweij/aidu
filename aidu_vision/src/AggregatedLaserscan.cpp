#include <ros/ros.h>
#include <aidu_vision/AggregatedLaserscan.h>
#include <aidu_vision/DistanceSensors.h>
#include <sensor_msgs/LaserScan.h>
#include <math.h>

using namespace aidu;

vision::AggregatedLaserscan::AggregatedLaserscan() : core::Node::Node() {
    
    // Subscribe to topics for laserscan and distance information
    this->laserscanSubscriber = nh->subscribe("/laserscan", 1, &vision::AggregatedLaserscan::laserscanCallback, this);
    this->distanceSubscriber = nh->subscribe("/sensors", 1, &vision::AggregatedLaserscan::distanceCallback, this);
    
    this->scanPublisher = nh->advertise<sensor_msgs::LaserScan>("/scan", 1);
    
    // = nh->advertise<sensor_msgs::LaserScan>("/scan2",1);

}

void vision::AggregatedLaserscan::laserscanCallback(const sensor_msgs::LaserScan::ConstPtr& scanmsg) {
    
    // Create new laserscan
    sensor_msgs::LaserScan newLaserScan;
    newLaserScan.ranges.resize(scanmsg->ranges.size());
    newLaserScan.ranges[0] = scanmsg->ranges[0];
    newLaserScan.range_max = scanmsg->range_max;
    newLaserScan.range_min = scanmsg->range_min;
    newLaserScan.angle_increment = scanmsg->angle_increment;
    newLaserScan.angle_max = scanmsg->angle_max;
    newLaserScan.angle_min = scanmsg->angle_min;
    newLaserScan.scan_time = scanmsg->scan_time;
    newLaserScan.header.frame_id = scanmsg->header.frame_id;
    newLaserScan.intensities = scanmsg->intensities;
    
    // Try to smart fill NaN ranges
    double startAverage = 0.0;
    unsigned int startIndex = 0;
    for (unsigned int i=1; i<scanmsg->ranges.size(); i++) {
        if (isnan(scanmsg->ranges[i]) && startIndex == 0) {
            startIndex = i - 1;
            startAverage = scanmsg->ranges[i-1];
        } else if(!isnan(scanmsg->ranges[i]) && startIndex != 0) {
            double endAverage = scanmsg->ranges[i];
            unsigned int endIndex = i;
            //ROS_INFO("%d - %f  =>  %d - %f", startIndex, startAverage, endIndex, endAverage);
            //double step = (endAverage - startAverage) / (endIndex - startIndex);
            double maxAverage = std::max(startAverage, endAverage);
            for(unsigned int j=startIndex + 1; j<endIndex; j++) {
                newLaserScan.ranges[j] = maxAverage; // startAverage + step * (j-startIndex);
                //ROS_INFO("  %d = %f", j, newLaserScan.ranges[j]);
            }
            startIndex = 0;
        } else {
            newLaserScan.ranges[i] = scanmsg->ranges[i];
        }
    }
    
    // See very far for any missed ranges
    for (unsigned int i=0; i<scanmsg->ranges.size(); i++) {
        if (isnan(newLaserScan.ranges[i]) || newLaserScan.ranges[i] == 0.0) {
            newLaserScan.ranges[i] = newLaserScan.range_max - 0.001;
        }
    }
    
    // Publish laser scan
    this->scanPublisher.publish(newLaserScan);
    
}

void vision::AggregatedLaserscan::distanceCallback(const aidu_vision::DistanceSensors::ConstPtr& distmsg) {
    
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "aggregatedlaserscan");
    vision::AggregatedLaserscan al;
    al.spin();
    return 0;
}

