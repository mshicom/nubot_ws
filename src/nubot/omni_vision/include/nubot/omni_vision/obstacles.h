#ifndef __NUBOT_VISION_OBSTACLES_H_
#define __NUBOT_VISION_OBSTACLES_H_


#include <opencv2/opencv.hpp>
#include "nubot/core/core.hpp"
#include "nubot/omni_vision/scanpoints.h"
#include "nubot/omni_vision/transfer.h"
#include "nubot/omni_vision/MTTracker.h"
#include "nubot/omni_vision/fieldinformation.h"

//Macro definition for color segmentation
#define VISION_COLORSEGMENT_YELLOW	0	//Yellow represent for football
#define VISION_COLORSEGMENT_BLACK	1	//green represent for ground
#define VISION_COLORSEGMENT_GREEN	2	//green represent for ground
#define VISION_COLORSEGMENT_UNKNOWCOLOR	3	//Unknown color

namespace nubot
{ 

const int    OBS_MINDARKNESS_CONST  = 50;
const double OBS_WEIGHT_THRES_CONST = 0.18;
const double OBS_RADIUS_CONST       = 25;
const double OBS_PARTITION_CONST    = 10;
const int    OBS_MAXDISTANCE_CONST  = 1000;
const int    OBS_NUMBER_CONST       = 50;
const int    OBS_VALUABLENUBMBER__CONST=10;

class Obstacles
{
public:
    Obstacles(ScanPoints & _scanpts,Transfer & _trans);
    void  process(cv::Mat & _segment_result,DPoint & _robot_loc,Angle & _robot_head);
    void detectblacks(std::vector<DPoint2i> & _pts,std::vector<uchar> & _ColorY,std::vector<DPoint2i> & _black_pts,std::vector<bool> & obs_segment);
	void getweights();
    void show_obstacles();
    void getobstacles();
    void  filterObstacles(DPoint & _robot_loc,Angle & _robot_head);

public:
    ScanPoints  * scanpts_;
    Transfer    * transfer_;
    FieldInformation field_info_;
	std::vector< std::vector<DPoint2i> > black_pts_;
	std::vector<double> weight_;
	std::vector<double> distance_;
    std::vector<PPoint>   real_obstacles_;
    std::vector<DPoint>   world_obstacles_;
    std::vector<obs_info> obs_measure_;
    std::vector< std::vector<bool> > obs_segments_;


private:
	int obsthres_;
	uchar miniY_;
	uchar maxY_;
	double interval_radian_;
};



}
#endif  //!__NUBOT_VISION_OBSTACLES_H_

