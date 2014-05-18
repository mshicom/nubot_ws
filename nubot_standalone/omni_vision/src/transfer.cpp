#include "nubot/omni_vision/transfer.h"
#include <fstream>

using namespace nubot;

Transfer::Transfer(const char* calinfo, Omni_Image & _omni_img)
{
	omni_img_=&_omni_img;
    width_ =omni_img_->get_width();
    height_=omni_img_->get_height();
	std::ifstream findispictoworld(calinfo);
    real_distances_=new double[width_*height_];
    memset(real_distances_,0,width_*height_*sizeof(double));
	double tempforcalib;
    for (int i=0;i<height_;i++)
	{
        for (int j=0;j<width_;j++)
		{
			findispictoworld>>tempforcalib;
            *(real_distances_+i*width_+j)=tempforcalib;
		}
	}
	findispictoworld.close();	
}

double   
Transfer::realdistance(const DPoint2i & img_coor)
{
    DPoint2d temptrans=DPoint2d(img_coor)-omni_img_->get_big_roi().center_;
    double pixel_dis=temptrans.norm();
    return 1.7368441469469196*tan(0.0070288129501232368*pixel_dis)*100;
  //  return real_distances_[int(img_coor.y_)*width_+int(img_coor.x_)];
}

vector<double> 
Transfer::realdistance(const std::vector<DPoint2i>& img_coor)
{
	size_t numtrans=img_coor.size();
    vector<double> distances;
	distances.reserve(numtrans);
	for(size_t i=0; i<numtrans; i++)
		distances.push_back(realdistance(img_coor[i]));
	return distances;
}


//!  get the real coordinate of object from image coordinate
PPoint
Transfer::realcoordinates(const DPoint2i & img_coor)
{
    DPoint2d temptrans=DPoint2d(img_coor)-omni_img_->get_big_roi().center_;
	return PPoint(temptrans.angle(),realdistance(img_coor));
}

vector<PPoint>
Transfer::realcoordinates(const std::vector<DPoint2i>& img_coor)
{
	size_t numtrans=img_coor.size();
	vector<PPoint> relwhite;
	relwhite.reserve(numtrans);
	for(size_t i=0; i<numtrans; i++)
	  relwhite.push_back(realcoordinates(img_coor[i]));
	return relwhite;
}
//!  get the robot coordinate of object from global world coordinate
PPoint 
Transfer::realcoordinates(const DPoint & world_coor,const DPoint & location ,const Angle & facing)
{
	DPoint pt=world_coor-location;
	PPoint pts(pt);
	return PPoint(pts.angle_-facing,pts.radius_);
}

//!  get the robot coordinates of object from global world coordinates
vector<PPoint> 
Transfer::realcoordinates(const std::vector<DPoint>& world_coor,const DPoint & location ,const Angle & facing)
{
	size_t numtrans=world_coor.size();
	vector<PPoint> relwhite;
	relwhite.reserve(numtrans);
	for(size_t i=0; i<numtrans; i++)
		relwhite.push_back(realcoordinates(world_coor[i],location,facing));
	return relwhite;
}

DPoint 
Transfer::worldcoordinates(const PPoint & polar_coor,const DPoint & location ,const Angle & facing)
{
	PPoint pt(polar_coor.angle_+facing,polar_coor.radius_);
	return (location+DPoint(pt));
}

vector<DPoint> 
Transfer::worldcoordinates(const vector<PPoint> & polar_coor,const DPoint & location ,const Angle & facing)
{
	size_t numtrans=polar_coor.size();
	vector<DPoint> world_pts;
	world_pts.reserve(numtrans);
	for(size_t i=0; i< numtrans; i++)
        world_pts.push_back(worldcoordinates(polar_coor[i],location,facing));
	return world_pts;
}

DPoint 
Transfer::worldcoordinates(const DPoint2i & img_coor,const DPoint & location ,const Angle & facing)
{
	PPoint pt=realcoordinates(img_coor);
	return worldcoordinates(pt,location,facing);
}

vector<DPoint> 
Transfer::worldcoordinates(const vector<DPoint2i> & img_coor,const DPoint & location ,const Angle & facing)
{
	
	size_t numtrans=img_coor.size();
	vector<DPoint> world_pts;
	world_pts.reserve(numtrans);
	for(size_t i=0; i< numtrans; i++)
		world_pts.push_back(worldcoordinates(img_coor[i],location,facing));
	return world_pts;
}
vector<PPoint> 
Transfer::rotate(const vector<PPoint>& robot_coor,const Angle & ang)
{
	size_t numtrans=robot_coor.size();
	vector<PPoint> polar_pts;
	polar_pts.resize(numtrans);
	for(size_t i=0; i< numtrans; i++)
		 polar_pts[i]=PPoint(robot_coor[i].angle_+ang,robot_coor[i].radius_);
	return polar_pts;
}
