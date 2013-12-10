
#include <aidu_mobile_base/odometry.h>
#include <aidu_mobile_base/State.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include <algorithm>

#define BOUND(x,y) std::max(std::min(x, y), -y)

using namespace aidu;

mobile_base::Odometry::Odometry(){
    stateSubscriber = nh->subscribe("/base/state", 5, &mobile_base::Odometry::processState, this);
    odometryPublisher = nh->advertise<nav_msgs::Odometry>("/odom", 1);
}

mobile_base::Odometry::~Odometry() {
    
}

void mobile_base::Odometry::processState(const aidu_mobile_base::State::ConstPtr& msg) {
    
    ros::Time currentTime = ros::Time::now();
    
    double dLeftWheel = msg->leftpos - previousLeftWheelPosition;
    double dRightWheel = msg->rightpos - previousRightWheelPosition;
    
    double r = (dLeftWheel + dRightWheel) / 2.0;
    double th = msg->angle;
    
    double dx = cos(th) * r;
    double dy = sin(th) * r;
    
    double v = (msg->leftspeed + msg->rightspeed) / 2.0;
    double vx = cos(th) * v;
    double vy = sin(th) * v;
    double vtheta = (th - theta) / (currentTime - previousTime).toSec();
    
    x += dx;
    y += dy;
    theta = th;
    
    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = currentTime;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odometryBroadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = currentTime;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vtheta;

    //publish the message
    odometryPublisher.publish(odom);
    
    // Store values for next iteration
    previousTime = ros::Time::now();
    previousLeftWheelPosition = msg->leftpos;
    previousRightWheelPosition = msg->rightpos;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "odometry");
    mobile_base::Odometry odometry;
    odometry.spin();
    return 0;
}