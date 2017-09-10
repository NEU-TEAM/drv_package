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
#ifdef ASTRA // use astra s
float dx_optic_to_link = 0.02;
float dy_optic_to_link = 0.01025;
float dz_optic_to_link = 0.035;
#else // use xtion
float dx_optic_to_link = 0;
float dy_optic_to_link = 0.01;
float dz_optic_to_link = 0.02;
#endif

float dx_link_to_pitch = 0;
#ifdef ASTRA
float dy_link_to_pitch = 0.0125;
#else
float dy_link_to_pitch = 0.00375;
#endif
float dz_link_to_pitch = 0.027;

float dx_pitch_to_yaw = 0.019;
float dy_pitch_to_yaw = 0;
float dz_pitch_to_yaw = 0.0773;

double dx_yaw_to_base_ = 0.01;
double dy_yaw_to_base_ = 0.0;
double dz_yaw_to_base_ = 1.002;


float pitch_offset_ = 93; // offset for pitch which was got from IMU

int servoPitch_ = 0; // judge if the servo pitch value is changing

// roslaunch param
std::string cameraLinkFrameID_ = "/vision_link";
std::string cameraPitchFrameID_ = "/camera_pitch_frame"; // y along the axis of pitch servo to the right, x toward front, z up, orign projection at the diameter of rotation plate
std::string cameraYawFrameID_ = "/camera_yaw_frame"; // y to the right, x to the front, z up, orign at the bottom of support
std::string baseLinkFrameID_ = "/base_link";
std::string cameraOpticalFrameID_ = "/openni_rgb_optical_frame";
std::string visionNameSpaceID_ = "/vision";

MoveMean mm(50); // the value indicate the strengh to stable the camera.

void configCallback(drv_tf::tfConfig &config, uint32_t level)
{
//    pitch_offset_ = config.camera_pitch_offset_cfg;
//    dz_yaw_to_base_ = config.camera_to_base_height_cfg;
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
//		static tf2_ros::TransformBroadcaster br_optic_to_link;
//		geometry_msgs::TransformStamped transformStamped1;
//		image_msg->header.stamp = image_msg->header.stamp;
//		transformStamped1.header.frame_id = cameraLinkFrameID_;
//		transformStamped1.child_frame_id = cameraOpticalFrameID_;
//		transformStamped1.transform.translation.x = dx_optic_to_link;
//		transformStamped1.transform.translation.y = dy_optic_to_link;
//		transformStamped1.transform.translation.z = dz_optic_to_link;
//		tf2::Quaternion q;
//		q.setRPY(3*M_PI_2 , 0, 3*M_PI_2);
//		transformStamped1.transform.rotation.x = q.x();
//		transformStamped1.transform.rotation.y = q.y();
//		transformStamped1.transform.rotation.z = q.z();
//		transformStamped1.transform.rotation.w = q.w();
//		br_optic_to_link.sendTransform(transformStamped1);

    static tf2_ros::TransformBroadcaster br_link_to_pitch;
    geometry_msgs::TransformStamped transformStamped2;
    transformStamped2.header.stamp = image_msg->header.stamp;
    transformStamped2.header.frame_id = cameraPitchFrameID_;
    transformStamped2.child_frame_id = cameraLinkFrameID_;
    transformStamped2.transform.translation.x = dx_link_to_pitch;
    transformStamped2.transform.translation.y = dy_link_to_pitch;
    transformStamped2.transform.translation.z = dz_link_to_pitch;
    tf2::Quaternion q2;
    q2.setRPY(0 , pitch_, 0);
    q2.setY(- q2.y());
    transformStamped2.transform.rotation.x = q2.x();
    transformStamped2.transform.rotation.y = q2.y();
    transformStamped2.transform.rotation.z = q2.z();
    transformStamped2.transform.rotation.w = q2.w();
    br_link_to_pitch.sendTransform(transformStamped2);

    static tf2_ros::TransformBroadcaster br_pitch_to_yaw;
    geometry_msgs::TransformStamped transformStamped3;
    transformStamped3.header.stamp = image_msg->header.stamp;
    transformStamped3.header.frame_id = cameraYawFrameID_;
    transformStamped3.child_frame_id = cameraPitchFrameID_;
    transformStamped3.transform.translation.x = dx_pitch_to_yaw;
    transformStamped3.transform.translation.y = dy_pitch_to_yaw;
    transformStamped3.transform.translation.z = dz_pitch_to_yaw;
    tf2::Quaternion q3;
    q3.setRPY(0 , 0, yaw_);
    transformStamped3.transform.rotation.x = q3.x();
    transformStamped3.transform.rotation.y = q3.y();
    transformStamped3.transform.rotation.z = q3.z();
    transformStamped3.transform.rotation.w = q3.w();
    br_pitch_to_yaw.sendTransform(transformStamped3);

    static tf2_ros::TransformBroadcaster br_yaw_to_base;
    geometry_msgs::TransformStamped transformStamped4;
    transformStamped4.header.stamp = image_msg->header.stamp;
#ifdef DEBUG_TRANS
    transformStamped4.header.frame_id = "/map";
#else
    transformStamped4.header.frame_id = baseLinkFrameID_;
#endif
    transformStamped4.child_frame_id = cameraYawFrameID_;
    transformStamped4.transform.translation.x = dx_yaw_to_base_;
    transformStamped4.transform.translation.y = dy_yaw_to_base_;
    transformStamped4.transform.translation.z = dz_yaw_to_base_;
    tf2::Quaternion q4;
    q4.setRPY(0, 0, 0);
    transformStamped4.transform.rotation.x = q4.x();
    transformStamped4.transform.rotation.y = q4.y();
    transformStamped4.transform.rotation.z = q4.z();
    transformStamped4.transform.rotation.w = q4.w();
    br_yaw_to_base.sendTransform(transformStamped4);
}


int main(int argc, char** argv){
    ros::init(argc, argv, "drv_tf");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    pnh.getParam("camera_optical_frame_id", cameraOpticalFrameID_);
    pnh.getParam("root_frame_id", baseLinkFrameID_);

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
