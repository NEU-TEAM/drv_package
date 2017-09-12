#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Header.h>
#include <std_msgs/UInt16MultiArray.h>

#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <dynamic_reconfigure/server.h>
#include <drv_tf/tfConfig.h>

#include "movemean.h"

//#define DEBUG_TRANS
//#define ASTRA

// camera frame rotation about world frame
float pitch_ = 0.0; // rotation angle -90 between camera optical frame and link is not included
float yaw_ = 0.0;


// distance between frame orign, in meter
float dx_link_to_pitch = 0; // here link refer camera link
#ifdef ASTRA
float dy_link_to_pitch = 0.0125;
#else
float dy_link_to_pitch = 0.05875;
#endif
float dz_link_to_pitch = 0.047;

float dx_pitch_to_yaw = 0.019;
float dy_pitch_to_yaw = 0;
float dz_pitch_to_yaw = 0.0773;

double dx_yaw_to_base_ = 0.01;
double dy_yaw_to_base_ = 0.0;
double dz_yaw_to_base_ = 1.002;


float pitch_offset_ = 90; // offset for pitch which was got from IMU

int servoPitch_ = 0; // judge if the servo pitch value is changing

// roslaunch param
std::string cameraLinkFrame_ = "/vision_link";
std::string cameraPitchFrame_ = "/camera_pitch_frame"; // y along the axis of pitch servo to the right, x toward front, z up, orign projection at the diameter of rotation plate
std::string cameraYawFrame_ = "/camera_yaw_frame"; // y to the right, x to the front, z up, orign at the bottom of support
std::string baseLinkFrame_ = "/base_link";

MoveMean mm(50); // the value indicate the strengh to stable the camera.

void configCallback(drv_tf::tfConfig &config, uint32_t level)
{
  pitch_offset_ = config.camera_pitch_offset_cfg;
  dx_yaw_to_base_ = config.base_to_root_x_cfg;
  dy_yaw_to_base_ = config.base_to_root_y_cfg;
  dz_yaw_to_base_ = config.base_to_root_z_cfg;
}

void servoCallback(const std_msgs::UInt16MultiArrayConstPtr &msg)
{
  // this callback should always active
  servoPitch_ = (msg->data[0]);
  yaw_ = (msg->data[1] - 90) * 0.01745;
}

void imuCallback(const std_msgs::Float32ConstPtr & msg)
{
  float imuPitchTemp = 0.0;
  pitch_ = msg->data - pitch_offset_; // notice that pitch_ < 0
  mm.getMoveMean(pitch_, imuPitchTemp);
  pitch_ = imuPitchTemp * 0.01745; // 3.14159 / 180.0
}

void imageCallback(const sensor_msgs::ImageConstPtr &image_msg)
{
  static tf2_ros::TransformBroadcaster br_link_to_pitch;
  geometry_msgs::TransformStamped ts_link_to_pitch;
  ts_link_to_pitch.header.stamp = image_msg->header.stamp;
  ts_link_to_pitch.header.frame_id = cameraPitchFrame_;
  ts_link_to_pitch.child_frame_id = cameraLinkFrame_;
  ts_link_to_pitch.transform.translation.x = dx_link_to_pitch;
  ts_link_to_pitch.transform.translation.y = dy_link_to_pitch;
  ts_link_to_pitch.transform.translation.z = dz_link_to_pitch;
  tf2::Quaternion q2;
  q2.setRPY(0 , pitch_, 0);
  q2.setY(- q2.y());
  ts_link_to_pitch.transform.rotation.x = q2.x();
  ts_link_to_pitch.transform.rotation.y = q2.y();
  ts_link_to_pitch.transform.rotation.z = q2.z();
  ts_link_to_pitch.transform.rotation.w = q2.w();
  br_link_to_pitch.sendTransform(ts_link_to_pitch);
  
  static tf2_ros::TransformBroadcaster br_pitch_to_yaw;
  geometry_msgs::TransformStamped ts_pitch_to_yaw;
  ts_pitch_to_yaw.header.stamp = image_msg->header.stamp;
  ts_pitch_to_yaw.header.frame_id = cameraYawFrame_;
  ts_pitch_to_yaw.child_frame_id = cameraPitchFrame_;
  ts_pitch_to_yaw.transform.translation.x = dx_pitch_to_yaw;
  ts_pitch_to_yaw.transform.translation.y = dy_pitch_to_yaw;
  ts_pitch_to_yaw.transform.translation.z = dz_pitch_to_yaw;
  tf2::Quaternion q3;
  q3.setRPY(0 , 0, yaw_);
  ts_pitch_to_yaw.transform.rotation.x = q3.x();
  ts_pitch_to_yaw.transform.rotation.y = q3.y();
  ts_pitch_to_yaw.transform.rotation.z = q3.z();
  ts_pitch_to_yaw.transform.rotation.w = q3.w();
  br_pitch_to_yaw.sendTransform(ts_pitch_to_yaw);
  
  static tf2_ros::TransformBroadcaster br_yaw_to_base;
  geometry_msgs::TransformStamped ts_yaw_to_base;
  ts_yaw_to_base.header.stamp = image_msg->header.stamp;
#ifdef DEBUG_TRANS
  transformStamped4.header.frame_id = "/map";
#else
  ts_yaw_to_base.header.frame_id = baseLinkFrame_;
#endif
  ts_yaw_to_base.child_frame_id = cameraYawFrame_;
  ts_yaw_to_base.transform.translation.x = dx_yaw_to_base_;
  ts_yaw_to_base.transform.translation.y = dy_yaw_to_base_;
  ts_yaw_to_base.transform.translation.z = dz_yaw_to_base_;
  tf2::Quaternion q4;
  q4.setRPY(0, 0, 0);
  ts_yaw_to_base.transform.rotation.x = q4.x();
  ts_yaw_to_base.transform.rotation.y = q4.y();
  ts_yaw_to_base.transform.rotation.z = q4.z();
  ts_yaw_to_base.transform.rotation.w = q4.w();
  br_yaw_to_base.sendTransform(ts_yaw_to_base);
}


int main(int argc, char** argv){
  ros::init(argc, argv, "drv_tf");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  
  pnh.getParam("root_frame_id", baseLinkFrame_); // param should not have /
  pnh.getParam("camera_frame_id", cameraLinkFrame_);
  pnh.getParam("pitch_offset_value", pitch_offset_);
  
  // get structure params, up to down
  pnh.getParam("dx_camera_to_pitch_value", dx_link_to_pitch);
  pnh.getParam("dy_camera_to_pitch_value", dy_link_to_pitch);
  pnh.getParam("dz_camera_to_pitch_value", dz_link_to_pitch);
  pnh.getParam("dx_pitch_to_yaw_value", dx_pitch_to_yaw);
  pnh.getParam("dy_pitch_to_yaw_value", dy_pitch_to_yaw);
  pnh.getParam("dz_pitch_to_yaw_value", dz_pitch_to_yaw);
  
  // set up dynamic reconfigure callback
  dynamic_reconfigure::Server<drv_tf::tfConfig> server;
  dynamic_reconfigure::Server<drv_tf::tfConfig>::CallbackType f;
  
  f = boost::bind(&configCallback, _1, _2);
  server.setCallback(f);
  
  ros::Subscriber sub_yaw = nh.subscribe<std_msgs::UInt16MultiArray> ("servo", 1, servoCallback);
  ros::Subscriber sub_acc = nh.subscribe("camera_pitch", 1, imuCallback);
  
  ros::NodeHandle rgb_nh(nh, "rgb");
  ros::NodeHandle rgb_pnh(pnh, "rgb");
  image_transport::ImageTransport it_rgb_sub(rgb_nh);
  image_transport::TransportHints hints_rgb("compressed", ros::TransportHints(), rgb_pnh);
  image_transport::Subscriber sub_rgb = it_rgb_sub.subscribe("/vision/rgb/image_rect_color", 1, imageCallback, hints_rgb);
  
  
  ROS_INFO("TF initialized.");
  
  while (ros::ok())
  {
    ros::spinOnce();
  }
  return 0;
};
