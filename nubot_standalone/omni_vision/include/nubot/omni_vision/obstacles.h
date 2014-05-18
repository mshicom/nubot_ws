#ifndef __NUBOT_VISION_OBSTACLES_H_
#define __NUBOT_VISION_OBSTACLES_H_


#include <opencv2/opencv.hpp>
#include "nubot/core/core.hpp"
#include "nubot/omni_vision/scanpoints.h"
#include "nubot/omni_vision/transfer.h"

namespace nubot
{ 

const int    OBS_MINDARKNESS_CONST  = 50;
const double OBS_WEIGHT_THRES_CONST = 0.18;
const double OBS_RADIUS_CONST       = 25;
const double OBS_PARTITION_CONST    = 10;
const int    OBS_MAXDISTANCE_CONST  = 1000;
const int    OBS_NUMBER_CONST       = 50;

class Obstacles
{
public:
    Obstacles(ScanPoints & _scanpts,Transfer & _trans);
	int  process();
	void detectblacks(std::vector<DPoint2i> & _pts,std::vector<uchar> & _ColorY,std::vector<DPoint2i> & _black_pts);
	void getweights();
	void show();
	int  getobstacles();
	void filter();

public:
    ScanPoints  * scanpts_;
    Transfer    * transfer_;
	std::vector< std::vector<DPoint2i> > black_pts_;
	std::vector<double> weight_;
	std::vector<double> distance_;
	std::vector<PPoint> obstacles_;

public:
	int obsthres_;
	uchar miniY_;
	uchar maxY_;
	double interval_radian_;
};



}
#endif  //!__NUBOT_VISION_OBSTACLES_H_

