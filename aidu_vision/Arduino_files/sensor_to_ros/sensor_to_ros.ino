/*
 * Rosserial sensor value publisher
 * Publishes the value of the sensors to ros
 * Sensor types: Sharp 041SK, Sharp GP2D12. Sharp 2Y0A02 and HC-SR04 (Ultrasonic)
 *
 * Usage: 
 * Run roscore
 * Type in terminal: rosrun rosserial_python serial_node.py /dev/ttyACM#
 * where # is the number of the serial port the Arduino is attached to
 * Call the topic with: rostopic echo sensors
 */
 
 
#include <ros.h>
#include <aidu_vision/DistanceSensors.h>
#include <math.h>

ros::NodeHandle  nh;

aidu_vision::DistanceSensors distance;
ros::Publisher sensor_publisher("sensors", &distance);


// Analog infrared defines
int sensorpin_ir_41 = 0; // analog pin used to connect the sharp 041SK
int sensorpin_ir_12 = 3; // analog pin used to connect the sharp GP2D12 
int sensorpin_ir_02 = 2; // analog pin used to connect the sharp 2Y0A02
// Ultrasonic defines
#define echoPin_clean 7
#define trigPin_clean 8 
#define echoPin_duct 12
#define trigPin_duct 13

void setup()
{
  nh.initNode();
  nh.advertise(sensor_publisher);
  
  pinMode(trigPin_clean, OUTPUT);
  pinMode(echoPin_clean, INPUT);
  pinMode(trigPin_duct, OUTPUT);
  pinMode(echoPin_duct, INPUT);
  
}

void loop()
{
  // Sharp 041SK
  // Range: 3 to 30 cm
  // Value: 30 to 300 (mm)
  // Attached to analog input 0
  //double sensor_value_ir_41 = analogRead(sensorpin_ir_41);
  //int dis_ir_41 = 27205.2633 * pow(sensor_value_ir_41, -1.0183);
  //distance.Back = dis_ir_41;
  
  // Sharp GP2D12
  // Range: 10 to 80 cm
  // Value: 100 to 800 (mm)
  // Attached to analog input 1
//  double sensor_value_ir_12 = analogRead(sensorpin_ir_12);
//  int dis_ir_12 = 38161.1739 * pow(sensor_value_ir_12, -0.9298) - 2936752.6632 * pow(sensor_value_ir_12,-2.2758995196);
//  distance.Frontmiddle = dis_ir_12;
  
  // Sharp 2Y0A02
  // Range: 20 to 150 cm
  // Value: 200 to 1500 (mm)
  // Attached to analog input 2
// double sensor_value_ir_02 = analogRead(sensorpin_ir_02);
// int dis_ir_02 = 100356.1342 * pow(sensor_value_ir_02, -0.97711388);
// int dir_ir_02 = dir_ir_02 + 0.11999*dir_ir_02 /*- 18.91585*/;
// distance.Left = dis_ir_02;
  
  
  // HC-SR04 ultrasonic clean
  // Range: 2 to 400 cm
  // Value: 20 to 4000 (mm)
  // Attached to digital 7 (echo, blue) and digital 8 (Trig, red) 
  
  float duration_clean[4], dis_us_clean;
  for(int i = 0; i < 3; i++){
    digitalWrite(trigPin_clean, LOW);
    delayMicroseconds(2);
  
    digitalWrite(trigPin_clean, HIGH);
    delayMicroseconds(10);
  
    digitalWrite(trigPin_clean, LOW);
    duration_clean[i] = pulseIn(echoPin_clean, HIGH);
    //delayMicroseconds(60000);
  } 
  duration_clean[3] = (((duration_clean[0] + duration_clean[1] + duration_clean[2])/3));
  dis_us_clean = (duration_clean[3] / 5.82 ) - 0.2979052982*(duration_clean[3] / 5.82 ) + 4.0456059645;
  dis_us_clean = dis_us_clean + 0.1*dis_us_clean - 10;
  distance.Frontright = dis_us_clean;
    //delayMicroseconds(60000);
  
  // HC-SR04 ultrasonic duct
  // Range: 2 to 400 cm
  // Value: 20 to 4000 (mm)
  // Attached to digital 12 (Echo, blue) and digital 13 (Trigger, red)
  float duration_duct[4], dis_us_duct;
  for(int i = 0; i < 3; i++){
    digitalWrite(trigPin_duct, LOW);
    delayMicroseconds(2);
  
    digitalWrite(trigPin_duct, HIGH);
    delayMicroseconds(10);
  
    digitalWrite(trigPin_duct, LOW);
    duration_duct[i] = pulseIn(echoPin_duct, HIGH);
    //delayMicroseconds(60000);
  } 
  duration_duct[3] = (((duration_duct[0] + duration_duct[1] + duration_duct[2])/3));
  dis_us_duct = (duration_duct[3] / 5.82 ) - 0.2979052982*(duration_duct[3] / 5.82 ) + 4.0456059645;
  dis_us_duct = dis_us_duct + 0.1*dis_us_duct - 10; 
  distance.Frontleft = dis_us_duct;
  
  
  
  // Publisher
  sensor_publisher.publish( &distance );
  nh.spinOnce();
  delay(500);
}

