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
#include <aidu_gui/Solenoid.h>
#include <math.h>

ros::NodeHandle  nh;

// Ultrasonic defines
#define echoPin_right 8
#define trigPin_right 9
#define echoPin_left 10
#define trigPin_left 11
#define echoPin_arm 12
#define trigPin_arm 13

//relay bord defines
int lades [4] = {-1,7,6,5}; 


//solenoid message callback
void messageCb( const aidu_gui::Solenoid& solenoid_msg){
  int pin_number=lades[solenoid_msg.solenoid_number];
  digitalWrite(pin_number, HIGH);
  delay(3000);
  digitalWrite(pin_number, LOW);
}

aidu_vision::DistanceSensors distance;
aidu_gui::solenoid solenoids;
ros::Publisher sensor_publisher("sensors", &distance);
ros::Subscriber<aidu_gui::Solenoid> sub("/solenoids", &messageCb );



void setup()
{
  nh.initNode();
  nh.advertise(sensor_publisher);
  nh.subscribe(sub);
  
  pinMode(trigPin_left, OUTPUT);
  pinMode(echoPin_left, INPUT);
  pinMode(trigPin_arm, OUTPUT);
  pinMode(echoPin_arm, INPUT);
  pinMode(trigPin_right, OUTPUT);
  pinMode(echoPin_right, INPUT);
  
  pinMode(7, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(5, OUTPUT);
}

void loop()
{
  
  // HC-SR04 ultrasonic left
  // Range: 2 to 400 cm
  // Value: 20 to 4000 (mm)
  // Attached to digital 7 (echo, blue) and digital 8 (Trig, red) 
  distance.Frontleft = ultrasonic(trigPin_left, echoPin_left);
  
  
  // HC-SR04 ultrasonic duct
  // Range: 2 to 400 cm
  // Value: 20 to 4000 (mm)
  // Attached to digital 12 (Echo, blue) and digital 13 (Trigger, red)
  distance.arm = ultrasonic(trigPin_arm, echoPin_arm);
  
    
  // HC-SR04 ultrasonic duct
  // Range: 2 to 400 cm
  // Value: 20 to 4000 (mm)
  // Attached to digital 12 (Echo, blue) and digital 13 (Trigger, red)
  distance.Frontright = ultrasonic(trigPin_right, echoPin_right);
  
  // Publisher
  sensor_publisher.publish( &distance );
  nh.spinOnce();
  delay(100);
}

/**
 * Measures distance from an ultrasonic sensor
 */
float ultrasonic(int triggerPin, int echoPin) {
  float duration[5], distance;
  for(int i = 0; i < 4; i++){
    digitalWrite(triggerPin, LOW);
    delayMicroseconds(2);
  
    digitalWrite(triggerPin, HIGH);
    delayMicroseconds(10);
  
    digitalWrite(triggerPin, LOW);
    duration[i] = pulseIn(echoPin, HIGH);
    delayMicroseconds(60000);
  } 
  duration[4] = (((duration[0] + duration[1] + duration[2] + duration[3])/4));
  distance = (duration[4] / 5.82 ) - 0.2979052982*(duration[4] / 5.82 ) + 4.0456059645;
  distance = distance + 0.1*distance - 10; 
  return distance;
}














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
  
//  // Sharp 2Y0A02
//  // Range: 20 to 150 cm
//  // Value: 200 to 1500 (mm)
//  // Attached to analog input 2
// double sensor_value_ir_02[5];
// for (int i=0;i<4;i++){
//   sensor_value_ir_02[i]= analogRead(sensorpin_ir_02);
//   delayMicroseconds(50000);
// }
// sensor_value_ir_02[4]=(sensor_value_ir_02[0]+sensor_value_ir_02[1]+sensor_value_ir_02[2]+sensor_value_ir_02[3])/4;
// int dis_ir_02 = 100356.1342 * pow(sensor_value_ir_02[4], -0.97711388);
// int dir_ir_02 = dir_ir_02 + 0.11999*dir_ir_02 /*- 18.91585*/;
// distance.Left = dis_ir_02;
// 
//  
//    // Sharp 2Y0A02
//  // Range: 20 to 150 cm
//  // Value: 200 to 1500 (mm)
//  // Attached to analog input 3
// double sensor_value_ir_022[5];
// for (int i=0;i<4;i++){
//   sensor_value_ir_022[i]= analogRead(sensorpin_ir_022);
//   delayMicroseconds(50000);
// }
// sensor_value_ir_022[4]=(sensor_value_ir_022[0]+sensor_value_ir_022[1]+sensor_value_ir_022[2]+sensor_value_ir_022[3])/4;
// int dis_ir_022 = 100356.1342 * pow(sensor_value_ir_022[4], -0.97711388);
// int dir_ir_022 = dir_ir_022 + 0.11999*dir_ir_022 /*- 18.91585*/;
// distance.Right = dis_ir_022;
//  
