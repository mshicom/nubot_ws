#ifndef __NUBOT_VISION_TRANSFER_H_
#define __NUBOT_VISION_TRANSFER_H_

#include <opencv2/opencv.hpp>
#include "nubot/core/core.hpp"
#include "nubot/omni_vision/omniimage.h"


namespace nubot
{ 

using std::vector;
class Transfer
{
public:
	//! read distance calibration  result and image infomation and location info;
    Transfer(std::string infopath,Omni_Image & _omni_img);
	
	//!  get the actual distance between the robot and object location from image coordinate 
    double   realdistance(const DPoint2i & img_coor);
	//!  get the actual distances between the robot and object locations from image coordinate 
	vector<double>  realdistance(const std::vector<DPoint2i>& img_coor);

    //! get the  image coordinates of the object from the robot coordinates
    vector<DPoint2i> imagecoordunates(const vector<PPoint> & real_coor);
    //! get the  image coordinate of the object from the robot coordinates
    DPoint2i imagecoordunates(const PPoint & real_coor);
    //! get the  image coordinates of the object from the  global world coordinates
    vector<DPoint2i> imagecoordunates(const vector<DPoint> & world_coor,const DPoint & location ,const Angle & facing);
    //! get the  image coordinate of the object from the  global world coordinate
    DPoint2i imagecoordunates(const DPoint & world_coor,const DPoint & location ,const Angle & facing);


	//!  get the robot coordinate of object from image coordinate
	PPoint realcoordinates(const DPoint2i & img_coor);
	//!  get the robot coordinates of object from image coordinates
	vector<PPoint> realcoordinates(const std::vector<DPoint2i>& img_coor);
	//!  get the robot coordinate of object from global world coordinate
	PPoint realcoordinates(const DPoint & world_coor,const DPoint & location ,const Angle & facing);
    //!  get the robot coordinates of object from global world coordinates
	vector<PPoint> realcoordinates(const std::vector<DPoint>& world_coor,const DPoint & location ,const Angle & facing);
	//!  robot coor rotate angle 
	vector<PPoint> rotate(const vector<PPoint>& robot_coor,const Angle & ang);

	//! get the global world coordinates of object from robot coordinate
	DPoint worldcoordinates(const PPoint & polar_coor,const DPoint & location ,const Angle & facing);
	//! get the global world coordinates of object from robot coordinates
	vector<DPoint> worldcoordinates(const vector<PPoint> & polar_coor,const DPoint & location ,const Angle & facing);
	//! get the  global world  coordinates of object from image coordinate
	DPoint worldcoordinates(const DPoint2i & img_coor,const DPoint & location ,const Angle & facing);
	//! get the  global world coordinates of object from image coordinates
	vector<DPoint> worldcoordinates(const vector<DPoint2i> & img_coor,const DPoint & location ,const Angle & facing);

    Omni_Image * omni_img_; //<-image information

private:
    /*! @brief the center is got by calibration the mirror*/
    DPoint2d center_;
    int height_;
    int width_;
    double para_a_;
    double para_b_;
};

}

#endif   //__NUBOT_VISION_TRANSFER_H_
