#include "goturn.h"
#include <math.h>

//minimum and maximum object area
const int MIN_OBJECT_AREA = 400;
const int MAX_OBJECT_AREA =  160000;

Goturn::Goturn(string test_proto, string caffe_model, int gpu_id,
               const bool do_train, const bool show_output)
    : regressor_(test_proto, caffe_model, gpu_id, do_train),
      tracker_(show_output)
{
    tracker_initialized_ = false;
}

void Goturn::goInit (Mat img, Rect gt)
{
    BoundingBox bbox_gt;
    bbox_gt.x1_ = gt.x;
    bbox_gt.y1_ = gt.y;
    bbox_gt.x2_ = gt.x + gt.width;
    bbox_gt.y2_ = gt.y + gt.height;

    tracker_.Init(img, bbox_gt, &regressor_);
    tracker_initialized_ = true;
}

Rect Goturn::goTrack(Mat img)
{
    BoundingBox bbox_estimate_uncentered;
    tracker_.Track(img, &regressor_, &bbox_estimate_uncentered);

    Rect dbox;
    dbox.x = bbox_estimate_uncentered.x1_;
    dbox.y = bbox_estimate_uncentered.y1_;
    dbox.width = bbox_estimate_uncentered.get_width();
    dbox.height = bbox_estimate_uncentered.get_height();

    return dbox;
}

bool Goturn::goProcess(Mat img_in, Rect gt, Mat &img_out, Rect &detection, std::vector<unsigned int> &mask_id)
{
    if (tracker_initialized_)
        {
            detection = goTrack(img_in);
        }
    else
        {
            Utilities::expandGt(gt, 3);
            goInit(img_in, gt);
            detection = gt;
        }

    Utilities::markImage(img_in, detection, img_out, mask_id);

    if (detection.area() < MIN_OBJECT_AREA ||  mask_id.size() < MIN_OBJECT_AREA ||
            detection.area() > MAX_OBJECT_AREA || mask_id.size() > MAX_OBJECT_AREA)
        {
            return false;
        }
    else
        return true;
}


