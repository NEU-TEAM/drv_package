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

#include <dynamic_reconfigure/server.h>
#include <drv_tf/tfConfig.h>

#define DEBUG_TRANS

// camera frame rotation about world frame
float pitch_ = 0.0; // rotation angle -90 between camera optical frame and link is not included
float yaw_ = 0.0;


// distance between frame orign, in meter
float dx_optic_to_link = 0;
float dy_optic_to_link = 0.01;
float dz_optic_to_link = 0.02;

float dx_link_to_pitch = 0;
float dy_link_to_pitch = 0.00375;
float dz_link_to_pitch = 0.027;

float dx_pitch_to_yaw = 0.019;
float dy_pitch_to_yaw = 0;
float dz_pitch_to_yaw = 0.0773;

float dx_yaw_to_base = 0;
float dy_yaw_to_base = 0;
double dz_yaw_to_base_ = 1.1;


float pitchtemp_ = 0.0;
int tol_ = 2; // the tolerance (degree) about camera shaking
int pitch_offset_ = 101; // offset for pitch which was got from IMU

// roslaunch param
std::string cameraLinkFrameID_ = "/camera_link_frame";
std::string cameraPitchFrameID_ = "/camera_pitch_frame"; // y along the axis of pitch servo to the right, x toward front, z up, orign projection at the diameter of rotation plate
std::string cameraYawFrameID_ = "/camera_yaw_frame"; // y to the right, x to the front, z up, orign at the bottom of support
std::string baseLinkFrameID_ = "/base_link";
std::string pointCloudSourceTopic_ = "/point_cloud";


void configCallback(drv_tf::tfConfig &config, uint32_t level)
{
    ROS_INFO("Reconfigure Request: offset: %d, height: %f.\n", config.camera_pitch_offset_cfg, config.ground_to_base_height_cfg);
    pitch_offset_ = config.camera_pitch_offset_cfg;
    dz_yaw_to_base_ = config.ground_to_base_height_cfg;
}

void pitchCallback(const std_msgs::Float32ConstPtr & msg)
{
    pitch_ = msg->data - pitch_offset_; // notice that pitch_ < 0
    if (fabs(pitch_ - pitchtemp_) > tol_ )
        pitchtemp_ = pitch_;
    pitch_ = pitchtemp_  * 0.01745; // / 180.0 * 3.14159
}

void yawCallback(const std_msgs::UInt16MultiArrayConstPtr &msg)
{
    // this callback should always active
    yaw_ = (msg->data[1] - 90) * 0.01745;
}

void tfCallback(const std_msgs::HeaderConstPtr & msg)
{
		/// transform indicate how to move or rotate parent frame to get to child frame

		// static transform initialize
		static tf2_ros::TransformBroadcaster br_optic_to_link;
		geometry_msgs::TransformStamped transformStamped1;

		transformStamped1.header.stamp = msg->stamp;
		transformStamped1.header.frame_id = cameraLinkFrameID_;
		transformStamped1.child_frame_id = "/openni_rgb_optical_frame";

		transformStamped1.transform.translation.x = dx_optic_to_link;
		transformStamped1.transform.translation.y = dy_optic_to_link;
		transformStamped1.transform.translation.z = dz_optic_to_link;
		tf2::Quaternion q;
		q.setRPY(3*M_PI_2 , 0, 3*M_PI_2);
		transformStamped1.transform.rotation.x = q.x();
		transformStamped1.transform.rotation.y = q.y();
		transformStamped1.transform.rotation.z = q.z();
		transformStamped1.transform.rotation.w = q.w();

		br_optic_to_link.sendTransform(transformStamped1);

    static tf2_ros::TransformBroadcaster br_link_to_pitch;
    geometry_msgs::TransformStamped transformStamped2;

    transformStamped2.header.stamp = msg->stamp;
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

    transformStamped3.header.stamp = msg->stamp;
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
		transformStamped4.header.stamp = msg->stamp;
#ifdef DEBUG_TRANS
		transformStamped4.header.frame_id = "/map";
#else
		transformStamped4.header.frame_id = baseLinkFrameID_;
#endif
		transformStamped4.child_frame_id = cameraYawFrameID_;

		transformStamped4.transform.translation.x = dx_yaw_to_base;
		transformStamped4.transform.translation.y = dy_yaw_to_base;
		transformStamped4.transform.translation.z = dz_yaw_to_base_;
		tf2::Quaternion q4;
		q4.setRPY(0 , 0, 0);
		transformStamped4.transform.rotation.x = q4.x();
		transformStamped4.transform.rotation.y = q4.y();
		transformStamped4.transform.rotation.z = q4.z();
		transformStamped4.transform.rotation.w = q4.w();

		br_yaw_to_base.sendTransform(transformStamped4);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "broadcast_camera_frame");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

		// the first string is the variable name used by roslaunch, followed by variable value defined by roslaunch, and the default value
		pnh.param("camera_pitch_offset", pitch_offset_, pitch_offset_);
		pnh.param("ground_to_base_height", dz_yaw_to_base_, dz_yaw_to_base_);
		pnh.param("base_link_frame_id", baseLinkFrameID_, baseLinkFrameID_);

		// set up dynamic reconfigure callback
		dynamic_reconfigure::Server<drv_tf::tfConfig> server;
		dynamic_reconfigure::Server<drv_tf::tfConfig>::CallbackType f;

		f = boost::bind(&configCallback, _1, _2);
		server.setCallback(f);

		ros::Subscriber sub_acc = nh.subscribe("/camera_pitch", 3, pitchCallback);
		ros::Subscriber sub_yaw = nh.subscribe<std_msgs::UInt16MultiArray> ("servo", 1, yawCallback);
		ros::Subscriber sub = nh.subscribe<std_msgs::Header>("point_cloud/header", 3, tfCallback);

		 ROS_INFO("TF initialized.\n");

    while (ros::ok())
        {
            ros::spinOnce();
        }
    return 0;
};
