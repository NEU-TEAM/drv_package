#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>

#include <actionlib/server/simple_action_server.h>
#include <drv_msgs/VisionAction.h>

#include <drv_msgs/recognized_faces.h>

using namespace std;

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

public:
    VisionAction(std::string name) :
        as_(nh_, name, boost::bind(&VisionAction::executeCB, this, _1), false),
        action_name_(name)
    {
        param_target_set =   "/comm/param/control/target/is_set";
        param_target_label = "/comm/param/control/target/label";

        local_param_need_recog = "/vision/face/need_recognize";

        param_vision_feedback = "/comm/param/feedback/vision";

        success_ = false;
        running_mode_ = 0;
        status_ = 0;
        error_detail_ = 0;

        sub_face_ = nh_.subscribe<drv_msgs::recognized_faces>("face/recognized_faces", 1, &VisionAction::faceCB, this);
        sub_object_ = nh_.subscribe<geometry_msgs::PoseStamped>("grasp/pose", 1, &VisionAction::objectCB, this);

        as_.start();
    }

    ~VisionAction(void)
    {
    }

    void getStatus()
    {
        ros::param::get(param_vision_feedback, status_);
    }

    void resetStatus()
    {
        success_ = false;
        ros::param::set(param_target_set, false);
        ros::param::set(local_param_need_recog, false);
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
        resetStatus(); // first reset all param

        if (goal->mode == 0)
            {
                feedback_.status = 0;
                // publish the feedback
                as_.publishFeedback(feedback_);
                return;
            }
        else if (goal->mode == 1)
            {
                if (goal->target_label == "")
                    {
                        feedback_.status = 2;
                        as_.publishFeedback(feedback_);
                        return;
                    }
                else
                    {
                        ros::param::set(param_target_set, true);
                        ros::param::set(param_target_label, goal->target_label);
                        feedback_.status = 1;
                        as_.publishFeedback(feedback_);
                    }
            }
        else if (goal->mode == 2)
            {
                ros::param::set(local_param_need_recog, true);
                // face recognize
                feedback_.status = 1;
                as_.publishFeedback(feedback_);
            }
        else
            {
                feedback_.status = 2;
                as_.publishFeedback(feedback_);
                return;
            }

        // publish info to the console for the user
        ROS_INFO("%s: Executing...", action_name_.c_str());

        // start executing the action
        if (as_.isPreemptRequested() || !ros::ok())
            {
                ROS_INFO("%s: Preempted", action_name_.c_str());
                // set the action state to preempted
                as_.setPreempted();
                success_ = false;
            }
        else
            {
                getStatus();
                feedback_.status = status_;
//                feedback_.error_detail_ = status_detail_; // the detail is not usable for now
                as_.publishFeedback(feedback_);
            }

        if(success_)
            {
                if (running_mode_ == 1)
                    {
                        // in recognize object mode
                        result_.target_pose = target_pose_;
                    }
                else
                    {
                        // in recognize face mode
                        result_.names = names_;
                        result_.ids = name_ids_;
                    }

                ROS_INFO("%s: Succeeded", action_name_.c_str());
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
    string local_param_need_recog;

    // overall feedback
    string param_vision_feedback;

    // result
    bool success_;

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


