#ifndef SMOOTHSERVO_H
#define SMOOTHSERVO_H

#include <ros/ros.h>

#include <std_msgs/UInt16MultiArray.h>

class SmoothServo
{
public:
  SmoothServo();
  
  bool moveServoTo(int pitch, int yaw);
  void getCurrentServoAngle(int pitch, int yaw);
  
private:
  int pitch_temp;
  int yaw_temp;
  int step;
  
  ros::NodeHandle nh;
  ros::Publisher servoPubSearch_;
  
  bool smooth(std::vector<std::vector<int> > &path, int p, int y);
  
};

#endif // SMOOTHSERVO_H
