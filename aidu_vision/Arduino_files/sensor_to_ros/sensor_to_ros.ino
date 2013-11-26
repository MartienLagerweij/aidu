/*
 * Rosserial sensor value publisher
 * Publishes the value of the sensor to ros
 * Sensor types: Sharp 041SK, Sharp GP2D12 and !!NOGIETS!!
 *
 * Usage: 
 * Run roscore
 * Type in terminal: rosrun rosserial_python serial_node.py /dev/ttyACM#
 * where # is the number of the serial port the Arduino is attached to
 * Call the topic with: rostopic echo [$SENSOR_TYPE]_sensor
 */
 
 
#include <ros.h>
#include <aidu_vision/DistanceSensors.h>
#include <math.h>

ros::NodeHandle  nh;

aidu_vision::DistanceSensors distance;
ros::Publisher sensor_publisher("sensors", &distance);
//ros::Publisher sharp12_sensor("sharp12_sensor", &distance12);

int sensorpin_41 = 0; // analog pin used to connect the sharp 041SK
int sensorpin_12 = 1; // analog pin used to connect the sharp GP2D12            

void setup()
{
  nh.initNode();
  nh.advertise(sensor_publisher);
  //nh.advertise(sharp12_sensor);
}

void loop()
{
  double sensor_value41 = analogRead(sensorpin_41);
  //int dis41 = (1/sensor_value41) * 26874;  // oude calibratie van chris
  int dis41 = 27205.2633 * pow(sensor_value41, -1.0183);
  distance.Frontleft = dis41;
  
  double sensor_value12 = analogRead(sensorpin_12);
  int dis12 = 38161.1739 * pow(sensor_value12, -0.9298) + 2936752.6632 * pow(sensor_value12,-2.2758995196);
  distance.Frontright = dis12;
  
  sensor_publisher.publish( &distance );
  
  nh.spinOnce();
  delay(500);
}

