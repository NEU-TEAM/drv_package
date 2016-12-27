#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <drv_msgs/face_recognize.h>

#include "processface.h"

std::string prototxt = "/home/aicrobo/caffe/models/finetune_neu_face/neu_face_deploy.prototxt";
std::string caffemodel = "/home/aicrobo/caffe/models/finetune_neu_face/finetune_neu_face.caffemodel";
ProcessFace pf_(prototxt, caffemodel, 0, false);

bool recognize_face(drv_msgs::face_recognize::Request  &req,
                    drv_msgs::face_recognize::Response &res)
{
    size_t im = req.images_in.size();
    vector<int> result_id;
    for (size_t i = 0;i < im; i++)
        {
            cv_bridge::CvImagePtr src = cv_bridge::toCvCopy(req.images_in[i], sensor_msgs::image_encodings::BGR8);
            int id = 0;
            float result_trust = 0.0;
            pf_.processFace(src->image, id, result_trust);

            if (result_trust < 0.9)
                result_id.push_back(0);
            else
                result_id.push_back(id + 1); // result id start at 0, while known person id start at 1
        }
    res.face_label_ids = result_id;
    return true; // return bool is necessary for service
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "face_recognize_server");
    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("drv_face_service", recognize_face);
    ROS_INFO("Ready to recognize face.");
    ros::spin();

    return 0;
}