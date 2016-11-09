#include "targetlistener.h"

TargetListener::TargetListener()
{
    isTargetSet_ = false;
    targetLabel_ = "";

    param_target_set =   "/comm/target/is_set";
    param_target_label = "/comm/target/label";
}

void TargetListener::getTargetStatus(bool &is_tgt_set, string &tgt_label)
{
    if (ros::param::has(param_target_set))
        {
            ros::param::get(param_target_set, is_tgt_set);
            if (is_tgt_set)
                {
                    if (ros::param::has(param_target_label))
                        {
                            ros::param::get(param_target_label, tgt_label);
                        }
                    else
                        {
                            // TODO feedback lack of label
                            tgt_label = "user selected object";
                        }
                }
        }
    else
        is_tgt_set = false;
}
