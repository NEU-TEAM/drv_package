#include <ros/ros.h>

#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>

#include <actionlib/server/simple_action_server.h>
#include <drv_msgs/VisionAction.h>

#include <drv_msgs/recognized_faces.h>

using namespace std;

enum FeedbackType{f_wander, f_working, f_failed};
enum GoalType{g_none, g_object, g_face};

class VisionAction
{
protected:
    ros::NodeHandle nh_;
    // NodeHandle instance must be created before this line. Otherwise strange error may occur.
    actionlib::SimpleActionServer<drv_msgs::VisionAction> as_;
    std::string action_name_;
    // create messages that are used to published feedback/result
    drv_msgs::VisionFeedback feedback_;
    drv_msgs::VisionResult result_;

    ros::Subscriber sub_face_;
    ros::Subscriber sub_object_;
    ros::Publisher pub_info_;

public:
    VisionAction(std::string name) :
        as_(nh_, name, boost::bind(&VisionAction::executeCB, this, _1), false),
        action_name_(name)
    {
        param_target_set =   "/comm/param/control/target/is_set";
        param_target_label = "/comm/param/control/target/label";

        param_need_recognize_face = "/vision/face/need_recognize";

        param_vision_feedback = "/comm/param/feedback/vision";

        running_mode_ = 0;
        status_ = 0;
        error_detail_ = 0;

        sub_face_ = nh_.subscribe<drv_msgs::recognized_faces>("face/recognized_faces", 1, &VisionAction::faceCB, this);
        sub_object_ = nh_.subscribe<geometry_msgs::PoseStamped>("grasp/pose", 1, &VisionAction::objectCB, this);

        pub_info_ = nh_.advertise<std_msgs::String>("/comm/msg/vision/info", 1);

        as_.start();
    }

    ~VisionAction(void)
    {
    }

    void pubInfo(string info)
    {
        ROS_INFO(info.c_str());
        std_msgs::String msg;
        msg.data = info;
        pub_info_.publish(msg);
    }

    void getStatus()
    {
        ros::param::get(param_vision_feedback, status_);
    }

    void resetStatus()
    {
        ros::param::set(param_target_set, false);
        ros::param::set(param_need_recognize_face, false);
    }

    void faceCB(const drv_msgs::recognized_facesConstPtr &face)
    {
        name_ids_ = face->name_ids;
        names_ = face->names;
    }

    void objectCB(const geometry_msgs::PoseStampedConstPtr &ps)
    {
        target_pose_ = *ps;
    }

    void executeCB(const drv_msgs::VisionGoalConstPtr &goal)
    {
        if (goal->mode == g_none)
            {
                resetStatus();
                feedback_.status = f_wander;
                as_.publishFeedback(feedback_);
                return;
            }
        else if (goal->mode == g_object)
            {
                if (goal->target_label == "")
                    {
                        feedback_.status = f_failed;
                        as_.publishFeedback(feedback_);
                        return;
                    }
                else
                    {
                        ros::param::set(param_target_label, goal->target_label);
                        ros::param::set(param_target_set, true);
                        feedback_.status = f_working;
                        as_.publishFeedback(feedback_);
                    }
            }
        else if (goal->mode == g_face)
            {
                ros::param::set(param_need_recognize_face, true);
                // face recognize
                feedback_.status = f_working;
                as_.publishFeedback(feedback_);
            }
        else
            {
                feedback_.status = f_failed;
                as_.publishFeedback(feedback_);
                return;
            }

        // publish info to the console for the user
        pubInfo("Vision Action: Executing...");

        // start executing the action
        if (as_.isPreemptRequested() || !ros::ok())
            {
                pubInfo("Vision Action: Preempted.");

                feedback_.status = f_wander;
                as_.publishFeedback(feedback_);
                as_.setPreempted();
                return;
            }
        else
            {
                getStatus();
                feedback_.status = status_;
                // feedback_.error_detail_ = status_detail_; // the detail is not usable for now
                as_.publishFeedback(feedback_);
            }

        if(status_ == 3 && goal->mode != g_none)
            {
                if (goal->mode == g_object)
                    {
                        // in recognize object mode
                        result_.target_pose = target_pose_;
                    }
                else if (goal->mode == g_face)
                    {
                        // in recognize face mode
                        result_.names = names_;
                        result_.ids = name_ids_;
                    }

                pubInfo("Vision Action: Succeeded.");
                as_.setSucceeded(result_);
                resetStatus(); // if goal has been reached, reset.
            }
    }

private:
    int running_mode_;
    int status_;
    int error_detail_;

    // goal
    string param_target_set;
    string param_target_label;
    string param_need_recognize_face;

    // overall feedback
    string param_vision_feedback;

    // result
    geometry_msgs::PoseStamped target_pose_;
    vector<int> name_ids_;
    vector<std_msgs::String> names_;
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "drv_action");

    VisionAction action(ros::this_node::getName());
    ros::spin();

    return 0;
}


