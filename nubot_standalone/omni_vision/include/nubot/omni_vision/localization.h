#ifndef __NUBOT_VISION_LOCALIZATION_H_
#define __NUBOT_VISION_LOCALIZATION_H_


#include <opencv2/opencv.hpp>
#include "nubot/core/core.hpp"
#include "nubot/omni_vision/fieldinformation.h"
#include "nubot/omni_vision/optimise.h"
#include "nubot/omni_vision/odometry.h"


namespace nubot
{ 
class Localization
{

public:

    Localization(Optimise & _optimise,Odometry & _odometry);

	void update_odometry(DPoint & _delta_pos,Angle & _delta_ang,DPoint & _robot_pos,Angle & _robot_ang);

	void kalmanfilter(DPoint _delta_pos,Angle _delta_ang, DPoint _pos_visual,Angle _ang_visual,
                      DPoint2d _var_xy,double _var_phi,DPoint & _pos_odo,Angle & _facing_odo);

	void kalmanfilter(DPoint _delta_pos,Angle _delta_ang,DPoint & _pos_odo,Angle & _facing_odo);

	void process(const std::vector<double> & _weights,const std::vector<PPoint> & _white_pt,DPoint & _robotloction, Angle & _angle);

public:
    Optimise * opti_;
    Odometry * odometry_;
    FieldInformation fildinfo_;

private:
    double   variant_ang_;
	DPoint2d variant_pos_;
};


}
#endif  //!__NUBOT_VISION_LOCALIZATION_H_

