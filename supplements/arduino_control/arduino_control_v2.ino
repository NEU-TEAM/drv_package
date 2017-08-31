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

#include <Servo.h>

#include <Wire.h>
#include <JY901.h>

#include "FastLED.h"
#define NUM_LEDS 2
CRGB leds[NUM_LEDS][1]; // use 2 leds, each has 1 light

int error_flag = 0;

Servo p_sv; // for camera up and down
Servo y_sv; // for head turn left and right

std_msgs::Float32 pitch_msg;
ros::Publisher pub_pitch("camera_pitch", &pitch_msg);

void error_cb(const std_msgs::UInt16& msg) {
  error_flag = msg.data;
}

void servo_cb(const std_msgs::UInt16MultiArray& msg) {
  p_sv.write(msg.data[0]); // 0-180
  y_sv.write(msg.data[1]); // 0-180
}

ros::Subscriber<std_msgs::UInt16> sub_error("error", error_cb);
ros::Subscriber<std_msgs::UInt16MultiArray> sub_servo("servo", servo_cb);

void setup() {
  Wire.begin();
  Serial.begin(9600);

  FastLED.addLeds<NEOPIXEL, 6>(leds[0], 1); // front
  FastLED.addLeds<NEOPIXEL, 7>(leds[1], 1); // back
  
  nh.initNode();
  nh.advertise(pub_pitch);
  nh.subscribe(sub_error);
  nh.subscribe(sub_servo);
  
  p_sv.attach(12);
  y_sv.attach(13);
}

void loop() {
  float x_a = (float)JY901.stcAngle.Angle[0]/32768*180;
  float y_a = (float)JY901.stcAngle.Angle[1]/32768*180;
  float z_a = (float)JY901.stcAngle.Angle[2]/32768*180;
  pitch_msg.data = x_a;
  pub_pitch.publish(&pitch_msg);

  nh.spinOnce();
  delay(10);

  checkError(); // if error occur, show it using led
}

void checkError() {
  // warning
  if (error_flag == 1) {
    for(int i = 0; i < NUM_LEDS; i++) {
      leds[i][0] = CRGB::Yellow;
      FastLED.show();
    }
  }
  // error
  else if (error_flag == 2) {
    for(int i = 0; i < NUM_LEDS; i++) {
      leds[i][0] = CRGB::Red;
      FastLED.show();
    }
  }
  // fatal error
  else if (error_flag == 3) {
    for(int i = 0; i < NUM_LEDS; i++) {
      leds[i][0] = CRGB::Red;
      FastLED.show();
    }
    delay(300);
    for(int i = 0; i < NUM_LEDS; i++) {
      leds[i][0] = CRGB::Black;
      FastLED.show();
    }
    delay(290);
  }
  // normal
  else {
    for(int i = 0; i < NUM_LEDS; i++) {
      leds[i][0] = CRGB::Green;
      FastLED.show();
    }
  }
}


