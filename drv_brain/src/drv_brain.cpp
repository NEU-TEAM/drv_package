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

//Custom message
#include <drv_msgs/target_info.h>


#include <stdio.h>

using namespace std;

bool centralSwitch_ = true; // main switch

// publish servo initial position
ros::Publisher servoInitPub_;
bool servo_initialized_ = false;

// target properties
enum TargetType{t_null, t_onTable, t_onGround, t_onHead, t_onHand};
string targetTypeName[5] = {"in air", "on the table", "on the ground", "on the face", "in the hand"};
int tgtType_ = t_null;
string param_target_type = "/status/target/type";

// target status control
string targetLabel_ = "";
bool isTargetSet_ = false;

//target status feedback
bool foundTarget_ = false;

// servo position angle
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


void initializeServo(int pitch_angle, int yaw_angle)
{
    std_msgs::UInt16MultiArray array;
    array.data.push_back(pitch_angle);
    array.data.push_back(yaw_angle);
    servoInitPub_.publish(array);
}

void servoCallback(const std_msgs::UInt16MultiArrayConstPtr &msg)
{
    // this callback should always active
    int pitch = msg->data[0];
    int yaw = msg->data[1];

    ros::param::set(param_servo_pitch, pitch);
    ros::param::set(param_servo_yaw, yaw);
}

void targetCallback(const drv_msgs::target_infoConstPtr &msg)
{
    if (msg->label.data == " " || msg->label.data == "")
        {
            ROS_INFO("Target canceled.\n");
            tgtType_ = t_null;
            targetLabel_ = "";
            isTargetSet_ = false;
        }
    else
        {
            targetLabel_ = msg->label.data;
            ROS_INFO("Target set to be '%s'.\n", targetLabel_.c_str());

            isTargetSet_ = true;

            ros::param::set(param_target_type, tgtType_);
            ros::param::set("/target/label", targetLabel_);
        }
}

void searchCallback(const std_msgs::Int8ConstPtr &msg)
{
    if (modeType_ == m_search)
        {
            if (msg->data == -1)
                {
                    ROS_INFO("Search around didn't get target, continue searching...\n");
                    foundTarget_ = false;
                    ros::param::set(param_base_move, 1);
                }
            else if (msg->data == 0)
                {
                    ROS_INFO("Currently didn't find target, continue searching...\n");
                    foundTarget_ = false;
                    ros::param::set(param_base_move, -1);
                }
            else
                {
                    ROS_INFO("Searching found the %s!\n", targetLabel_.c_str());
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
                    ROS_INFO_THROTTLE(19,"Target lost!\n");
                    foundTarget_ = false;
                }
            else
                {
                    ROS_INFO_THROTTLE(19,"Tracking the target...\n");
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

		servoInitPub_ = nh.advertise<std_msgs::UInt16MultiArray>("servo", 1, true);

		// don't change the order without reason
		ros::Subscriber sub_servo = nh.subscribe<std_msgs::UInt16MultiArray> ("servo", 1, servoCallback);
		ros::Subscriber sub_tgt = nh.subscribe<drv_msgs::target_info>("recognize/target", 1, targetCallback);
		ros::Subscriber sub_sh = nh.subscribe<std_msgs::Int8>("status/search/feedback", 1, searchCallback);
		ros::Subscriber sub_tk = nh.subscribe<std_msgs::Bool>("status/track/feedback", 1, trackCallback);

		ROS_WARN("Deep Robot Vision system initialized!\n");

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

						// target infomation preparation before mode selection
						if (sub_tgt.getNumPublishers() == 0)
								{
										ROS_INFO_THROTTLE(71, "Target not set (no target publisher).\n");
										resetStatus();
								}

						// Initialize servo position
						if (!servo_initialized_)
								{
										ROS_INFO("Servo initialized.\n");
										initializeServo(70, 90);
										servo_initialized_ = true;
								}

						// get status infomation
						ros::spinOnce();

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
										ROS_INFO("Current mode: %s.\n", modeName[modeType_].c_str());
										modeTypeTemp_ = modeType_;
								}
				}

		return 0;
}

