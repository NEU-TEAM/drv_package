/* Author: Dong Zhipeng 2017/9/12
 *  This code is for NVG's Arduino Mega 2560 to connect and communicate with ROS 
 *  as well as report angle data, control the servos and display status with leds.
 *  
 *  Connection map
 *  Componts         Mega 2560
 *  JY61(TX)    ->   digital 19
 *  servo_pitch ->   digital 2
 *  servo_yaw   ->   digital 3
 */

#include <ros.h>

#if (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

#include <ros/time.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/UInt16MultiArray.h>

ros::NodeHandle  nh;

/* Servo control */
#include <VarSpeedServo.h>
const int servo_speed = 50; // 0-255
VarSpeedServo p_sv;
VarSpeedServo y_sv;

void servo_cb(const std_msgs::UInt16MultiArray& msg) {
  int p_v = msg.data[0];
  int y_v = msg.data[1];
  if (p_v < 1) p_v = 1;
  if (p_v > 179) p_v = 179;
  if (y_v < 1) y_v = 1;
  if (y_v > 179) y_v = 179;
  
  if (msg.data_length == 2) {
    p_sv.write(p_v, servo_speed, false);
    y_sv.write(y_v, servo_speed, true);
  }
  else if (msg.data_length == 3) {
    if (msg.data[2] > 0 && msg.data[2] < 255) {
      p_sv.write(p_v, msg.data[2], false);
      y_sv.write(y_v, msg.data[2], true);
    }
    else if (msg.data[2] >= 255) {
      p_sv.write(p_v, 255, false);
      y_sv.write(y_v, 255, true);
    }
  }
  else if (msg.data_length == 4) {
    if (msg.data[2] > 0 && msg.data[2] < 255) {
      p_sv.write(p_v, msg.data[2], false);
    }
    else if (msg.data[2] >= 255) {
      p_sv.write(p_v, 255, false);
    }
    if (msg.data[3] > 0 && msg.data[3] < 255) {
      y_sv.write(y_v, msg.data[3], true);
    }
    else if (msg.data[3] >= 255) {
      y_sv.write(y_v, 255, true);
    }
  }
}

ros::Subscriber<std_msgs::UInt16MultiArray> sub_servo("servo", servo_cb);


/* IMU */
#include <JY901.h>
float x_ang = 0.0; // roll angle
float y_ang = 0.0; // ptich angle
float z_ang = 0.0; // yaw angle
float offset_yaw = 0.0; // When y_sv = 90, record this value, suppose servo is accurate

std_msgs::Float32 pitch_msg;
ros::Publisher pub_pitch("camera_pitch", &pitch_msg);

//std_msgs::Float32 yaw_msg;
//ros::Publisher pub_yaw("camera_yaw", &yaw_msg);


void setup() {
  Serial.begin(57600);
  Serial1.begin(115200); // 18 TX, 19 RX

  nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.advertise(pub_pitch);
  nh.subscribe(sub_servo);
  
  p_sv.attach(2, 500, 2500);
  y_sv.attach(3, 500, 2500);

  while (Serial1.available()) 
  {
    JY901.CopeSerialData(Serial1.read()); //Call JY901 data cope function
  }
}

void loop() {
  while (Serial1.available()) 
  {
    JY901.CopeSerialData(Serial1.read()); //Call JY901 data cope function
  }
  x_ang = (float)JY901.stcAngle.Angle[0]/32768*180;
  y_ang = (float)JY901.stcAngle.Angle[1]/32768*180;
  z_ang = (float)JY901.stcAngle.Angle[2]/32768*180;

  pitch_msg.data = x_ang; // We make x axis as yaw
  pub_pitch.publish(&pitch_msg);
  
//  yaw_msg.data = 90.0 - z_ang - offset_yaw;
//  pub_yaw.publish(&yaw_msg);
  
  nh.spinOnce();
  delay(50);
}

