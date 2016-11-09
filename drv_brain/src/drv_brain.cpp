#include <ros/ros.h>
#include <ros/console.h>

#include <math.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int8.h>
#include <std_msgs/UInt16MultiArray.h>
#include <std_msgs/Int32MultiArray.h>

//Custom message
#include <drv_msgs/target_info.h>

#include <stdio.h>

#include "androidlistener.h"
#include "targetlistener.h"

using namespace std;

bool centralSwitch_ = true; // main switch

// target properties
bool targetSetTemp = false;
string param_target_label = "/vision/target/label";
enum TargetType{t_null, t_onTable, t_onGround, t_onHead, t_onHand};
string targetTypeName[5] = {"in air", "on the table", "on the ground", "on the face", "in the hand"};
int tgtType_ = t_null;
string param_target_type = "/status/target/type";


// target status control
string targetLabel_ = "";
bool isTargetSet_ = false;

//target status feedback
bool foundTarget_ = false;

// publish servo initial position
ros::Publisher servoPub_;
bool servo_initialized_ = false;

// servo position angle
int pitchAngle_ = 70;
int yawAngle_ = 90;
string param_servo_pitch = "/status/servo/pitch";
string param_servo_yaw = "/status/servo/yaw";

// robot movement control param
// 0: free move, 1: should go to next position, -1: hold
string param_base_move = "/status/base/move";


// mode control params
enum ModeType{m_wander, m_search, m_track};
int modeType_ = m_wander;
int modeTypeTemp_ = -1;
string modeName[3] = {"wandering", "searching", "tracking"};
string param_running_mode = "/status/running_mode";
ros::Publisher drvPubMode_; // vision system mode publisher

//general infomation publisher
ros::Publisher drvPubInfo_;


void pubServo(int pitch_angle, int yaw_angle)
{
    std_msgs::UInt16MultiArray array;
    array.data.push_back(pitch_angle);
    array.data.push_back(yaw_angle);
    servoPub_.publish(array);
}

void pubInfo(string info)
{
    ROS_INFO(info.c_str());
    std_msgs::String msg;
    msg.data = info;
    drvPubInfo_.publish(msg);
}

void teleOpCallback(const std_msgs::Int32MultiArrayConstPtr &msg)
{
    if (msg->data.empty())
        return;

    int pitch_temp = pitchAngle_ + msg->data[0];
    if (pitch_temp < 40 || pitch_temp > 130)
        return;
    else
        pitchAngle_ = pitch_temp;

    int yaw_temp = yawAngle_ - msg->data[1];
    if (yaw_temp < 0 || pitch_temp > 180)
        return;
    else
        yawAngle_ = yaw_temp;

    pubServo(pitchAngle_, yawAngle_);
}

void servoCallback(const std_msgs::UInt16MultiArrayConstPtr &msg)
{
    // this callback should always active
    pitchAngle_ = msg->data[0];
    yawAngle_ = msg->data[1];

    ros::param::set(param_servo_pitch, pitchAngle_);
    ros::param::set(param_servo_yaw, yawAngle_);
}


// TODO: change this to param
//void targetCallback(const drv_msgs::target_infoConstPtr &msg)
//{
//    if (msg->label.data == " " || msg->label.data == "")
//        {
//            pubInfo("Target canceled.");
//            tgtType_ = t_null;
//            targetLabel_ = "";
//            isTargetSet_ = false;
//            ros::param::set("/status/target/is_set", false);
//        }
//    else
//        {
//            targetLabel_ = msg->label.data;
//            string s = "Target set to be '" + targetLabel_ + "'.";
//            pubInfo(s);

//            isTargetSet_ = true;
//            ros::param::set("/status/target/is_set", true);

//            ros::param::set(param_target_type, tgtType_);
//            ros::param::set("/target/label", targetLabel_);
//        }
//}

void searchCallback(const std_msgs::Int8ConstPtr &msg)
{
    if (modeType_ == m_search)
        {
            if (msg->data == -1)
                {
                    pubInfo("Search around didn't get target, continue searching...");
                    foundTarget_ = false;
                    ros::param::set(param_base_move, 1);
                }
            else if (msg->data == 0)
                {
                    pubInfo("Currently didn't find target, continue searching...");
                    foundTarget_ = false;
                    ros::param::set(param_base_move, -1);
                }
            else
                {
                    pubInfo("Searching found the " + targetLabel_);
                    foundTarget_ = true;
                    ros::param::set(param_base_move, 0);
                }
        }
}


void trackCallback(const std_msgs::BoolConstPtr &msg)
{
    if (modeType_ == m_track)
        {
            if (!msg->data)
                {
                    ROS_INFO_THROTTLE(21, "Target lost!");
                    foundTarget_ = false;
                }
            else
                {
                    ROS_INFO_THROTTLE(21, "Tracking the target...");
                    foundTarget_ = true;
                }
        }
}


void resetStatus()
{
		tgtType_ = t_null;
		targetLabel_ = "";

		isTargetSet_ = false;
		foundTarget_ = false;

		modeType_ = m_wander;
		ros::param::set(param_base_move, 0);
}

int main(int argc, char **argv)
{
		ros::init(argc, argv, "drv_brain");

		ros::NodeHandle nh;
		ros::NodeHandle pnh("~");

		servoPub_ = nh.advertise<std_msgs::UInt16MultiArray>("servo", 1, true);
		drvPubMode_ = nh.advertise<std_msgs::String>("/comm/vision/mode", 1);
		drvPubInfo_ = nh.advertise<std_msgs::String>("/comm/vision/info", 1);

		// don't change the order without reason
		ros::Subscriber sub_servo_ctrl = nh.subscribe<std_msgs::Int32MultiArray>("/joy_teleop/servo", 2, teleOpCallback);
		ros::Subscriber sub_servo = nh.subscribe<std_msgs::UInt16MultiArray> ("servo", 1, servoCallback);
//		ros::Subscriber sub_tgt = nh.subscribe<drv_msgs::target_info>("recognize/target", 1, targetCallback);
		ros::Subscriber sub_sh = nh.subscribe<std_msgs::Int8>("status/search/feedback", 1, searchCallback);
		ros::Subscriber sub_tk = nh.subscribe<std_msgs::Bool>("status/track/feedback", 1, trackCallback);

		AndroidListener al;
		TargetListener tl;

		pubInfo("Deep Robot Vision system initialized!");

		while (ros::ok())
				{
						// main on/off control
						if (ros::param::has("/status/central/switch"))
								{
										bool temp = true;
										ros::param::get("/status/central/switch", temp);
										if (temp)
												{
														ROS_WARN_COND(!centralSwitch_, "Central switch is ON.\n");
												}
										centralSwitch_ = temp;
								}

						if (!centralSwitch_)
								{
										ROS_WARN_THROTTLE(31, "Central switch is OFF.\n");
										resetStatus();
										continue;
								}

						// Initialize servo position
						if (!servo_initialized_)
								{
										pubServo(70, 90);
										servo_initialized_ = true;
										ROS_INFO("Servo initialized.\n");
								}

						// get feedback from search and track to determine is target found
						ros::spinOnce();

						// get target if params were set
						tl.getTargetStatus(isTargetSet_, targetLabel_);
						ros::param::set(param_target_label, targetLabel_);
						if (isTargetSet_ != targetSetTemp)
								{
										if (isTargetSet_)
												{
														string s = "Target set to be '" + targetLabel_ + "'.";
														pubInfo(s);
												}
										else
												{
														pubInfo("Target canceled.");
														resetStatus();
												}
										targetSetTemp = isTargetSet_;
								}

						// if user select target on cellphone, publish the target
						al.publishOnceIfTargetSelected(isTargetSet_, foundTarget_);

						//mode selection, NOTICE that modeType_ should only be set by central control
						if (isTargetSet_)
								{
										if (foundTarget_)
												{
														modeType_ = m_track;
												}
										else
												{
														modeType_ = m_search;
												}
								}
						else
								{
										modeType_ = m_wander;
								}

						// set mode
						ros::param::set(param_running_mode, modeType_);
						if (modeType_ != modeTypeTemp_)
								{
										std_msgs::String mode_msg;
										mode_msg.data = modeName[modeType_];
										drvPubMode_.publish(mode_msg);
										ROS_INFO("Current mode: %s.\n", modeName[modeType_].c_str());
										modeTypeTemp_ = modeType_;
								}
				}

		return 0;
}

