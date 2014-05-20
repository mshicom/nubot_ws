#ifndef __NUBOT_VISION_BALL_H_
#define __NUBOT_VISION_BALL_H_

#include "nubot/omni_vision/colorsegment.h"
#include "nubot/omni_vision/transfer.h"
#include "nubot/omni_vision/omniimage.h"
#include "nubot/core/core.hpp"

namespace nubot
{ 
const int BALL_PREDICT_BIAS_CONST=60;

class BallFinder
{

public:
    ColorSegment *color_segment_;

public:

    ColorSegment::ImageArea ball_area_;
    BallFinder(const char* _ColorTablePath, Transfer & _transfer);
    ~BallFinder();
    bool Process(cv::Mat &_image,const DPoint & _location, const Angle & _anlge);
    bool evaluate_velocity(double & _distance_ball, double & _distance_robot, Angle & _difference_angle);

    DPoint get_ball_global_loc();
    PPoint get_ball_real_loc();
    DPoint get_ball_velocity();
    bool   is_velocity_known();

private:
    Transfer *  transfer_;
    std::vector<DPoint> ball_record_;
    std::vector<double> ball_time_;
    std::vector<DPoint> record_velocity_;

    DPoint ball_global_loc_;
    PPoint ball_real_loc_;
    DPoint ball_global_vec_;
    bool   ball_vec_known_;

};



}
#endif  //!__NUBOT_VISION_BALL_H_

