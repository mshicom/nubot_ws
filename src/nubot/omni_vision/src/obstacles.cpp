#include "nubot/omni_vision/obstacles.h"

using namespace nubot;

Obstacles::Obstacles(ScanPoints & _scanpts,Transfer & _trans)
{
	scanpts_=&_scanpts;	
	transfer_=& _trans;
	size_t numtrans=scanpts_->polar_pts_.size();
    obsthres_=60;
	interval_radian_=DOUBLEPI_CONSTANT/numtrans;
	weight_.resize(numtrans);
	distance_.resize(numtrans);
	black_pts_.resize(numtrans);
    obs_segments_.resize(numtrans);
    real_obstacles_.reserve(OBS_MAXDISTANCE_CONST);
    world_obstacles_.reserve(OBS_MAXDISTANCE_CONST);
    obs_measure_.reserve(OBS_MAXDISTANCE_CONST);
	for(size_t i=0 ; i<numtrans; i++)
    {
		black_pts_[i].reserve(scanpts_->polar_pts_[i].size());
        obs_segments_[i].reserve(scanpts_->polar_pts_[i].size());
    }

}

/**  @brief   the main function of obstacles detection
 *   @return  the number of obstacles detected
 *   @author  Dan Xiong
 *   @see     detectblacks,getweights,getobstacles
 *   @date    2013.12.27*/
void
Obstacles::process(cv::Mat & _segment_result,DPoint & _robot_loc,Angle & _robot_head)
{
   int numsobstacles(0);
   real_obstacles_.clear();
   size_t numtrans=scanpts_->polar_pts_.size();
   //uchar miniY=*min_element(_ColorY.begin(),_ColorY.end());
   //uchar miniY=*max_element(_ColorY.begin(),_ColorY.end());
   miniY_=255;maxY_=0;
   for(size_t i=0; i< numtrans ;i++)
   {
       obs_segments_[i].clear();
       std::vector<DPoint2i> & polar_pts= scanpts_->polar_pts_[i];
       std::vector<uchar>    & ColorY   = scanpts_->polar_pts_y_[i];
       size_t Line_Length=polar_pts.size();
	   for(size_t j = 0 ; j < Line_Length ; j++)
	   {
           if(ColorY[j]<miniY_) miniY_ = ColorY[j];
           if(ColorY[j]>maxY_)  maxY_  = ColorY[j];
           if(_segment_result.at<unsigned char>(polar_pts[j].y_,polar_pts[j].x_)==VISION_COLORSEGMENT_BLACK)
               obs_segments_[i].push_back(true);
           else
               obs_segments_[i].push_back(false);
	   }
   }
   if(miniY_>OBS_MINDARKNESS_CONST) 
       return  ;
   for(size_t i=0;i<numtrans;i++)
       detectblacks(scanpts_->polar_pts_[i],scanpts_->polar_pts_y_[i],black_pts_[i],obs_segments_[i]);

   getweights();
   getobstacles();
   filterObstacles(_robot_loc,_robot_head);
}
void 
Obstacles::detectblacks(std::vector<DPoint2i> & _pts,std::vector<uchar> & _ColorY,
                        std::vector<DPoint2i> & _black_pts,std::vector<bool> & obs_segment)
{
	_black_pts.clear();
	size_t Line_Length=_pts.size();
	for(size_t i=0; i < Line_Length; i++)  
	{
        if(_ColorY[i] !=0&&_ColorY[i]<obsthres_&& obs_segment[i])//miniY_+maxY_*(0.05+0.05*i/Line_Length))
			_black_pts.push_back(_pts[i]);	 
	}
}
void 
Obstacles::getweights()
{
	size_t numtrans=black_pts_.size();
	DPoint2d weight_center(0,0);
    Circle Image_ROI=scanpts_->omni_img_->get_big_roi();
	for(size_t i=0;i<numtrans;i++)
	{
		weight_[i]   = DBL_MIN;
		distance_[i] = DBL_MAX;
		std::vector<DPoint2i> & pts=black_pts_[i];
		size_t nums_black=pts.size();
		if(nums_black>0) 
		{
			weight_center=DPoint2d(0,0);
			for(size_t j=0 ;j<nums_black;j++)
				weight_center+=DPoint2d(pts[j]);
			weight_center=1.0/nums_black*weight_center;
            double center_dis=weight_center.distance(Image_ROI.center_);

			weight_[i]=sqrt((double(nums_black)/scanpts_->polar_pts_[i].size())*
                (Image_ROI.radius_-center_dis)/Image_ROI.radius_);
			if(weight_[i]>OBS_WEIGHT_THRES_CONST)
				distance_[i]=transfer_->realdistance(pts[0]);	
		}
	}

}

void
Obstacles::getobstacles()
{
	int numtrans=weight_.size();
    int rulerupper,rulerlower;
	int numobstacles(0);
	for (int j=0; j<numtrans; ++j)
	{
		int closestobs(0);
        for (int i=0; i<numtrans; ++i)
		{
			if(distance_[i]<distance_[closestobs])
				closestobs = i;
		}
		double min_distance = distance_[closestobs];
		if (min_distance>OBS_MAXDISTANCE_CONST) break;
		/*from TU/E code*/
		int flag(0);
		for (rulerupper=closestobs;rulerupper<closestobs+numtrans;++rulerupper) 
		{
			if (std::abs(distance_[rulerupper%numtrans]-min_distance)>=OBS_PARTITION_CONST)
            {
			   if (flag==1) break; else flag++;
			} 
			else {flag=0;}
		}
		flag=0;
		for (rulerlower=closestobs;rulerlower>closestobs-numtrans;--rulerlower) 
		{
			int ind = rulerlower%numtrans;
			if (ind<0) ind+=numtrans;
			if (std::abs(distance_[ind%numtrans]-min_distance)>=OBS_PARTITION_CONST) 
            {
                   if (flag==1) break; else flag++;
			}
			else {flag=0;}
		}
		 
		/*compute obstacle parameters*/
		double obsphi   = (rulerupper+rulerlower)/2*interval_radian_;
		double obsdist  = min_distance;
		double obswidth = rulerupper-rulerlower-2;

		/*check size of obstacle(s)*/
		double width = obswidth*interval_radian_*obsdist;
		//determine amount of obstacles to split obstacle blob
		int n = int(0.5+width/2.0/OBS_RADIUS_CONST*1.1);
		//clip amount of subdivisions
		if (n>4) n=4;
		if (n<1) n=1;

		//determine new upper and lower limit (used for n>1)
		double philower = rulerlower*interval_radian_+1.*OBS_RADIUS_CONST/obsdist;
		double phiupper = rulerupper*interval_radian_-1.*OBS_RADIUS_CONST/obsdist;

		for (int i=0; i<n; ++i)
		{
			if (numobstacles<OBS_NUMBER_CONST)
			{
				//determine center of obstacle in case of n>1
                //! the angle has a bias
				if (n>1) obsphi = philower+(phiupper-philower)*i/(n-1.0);			
                real_obstacles_.push_back(PPoint(Angle(obsphi),obsdist+OBS_RADIUS_CONST));
				++numobstacles;
			}
		}
		
		for (int k=rulerlower-2;k<rulerupper+2;++k) 
		{
			int ind = k%numtrans;
			if (ind<0) ind+=numtrans;
			distance_[ind] = DBL_MAX;
		}
	}
}

void
Obstacles::filterObstacles(DPoint & _robot_loc, Angle & _robot_head)
{
    if(real_obstacles_.empty())
        return ;
    static MTTracker tracker;
    std::vector<obs_info> & measure_datas=obs_measure_;
    measure_datas.clear();
    obs_info mt;
    std::vector<DPoint> obscount =transfer_->worldcoordinates(real_obstacles_,_robot_loc,_robot_head);
    int obs_nums_infiled=0;
    for(size_t j=0; j < obscount.size() ;j++)
    {
        if(field_info_.isInInterRect(obscount[j]) && obs_nums_infiled < OBS_VALUABLENUBMBER__CONST)
        {
            mt.polar_pt  = real_obstacles_[j];
            mt.world_pt  = obscount[j];
            mt.base=_robot_loc;
            Filter::CalcInformationMatrix(mt);
            measure_datas.push_back(mt);
            obs_nums_infiled++;
        }
    }
    tracker.MeasureAssociate(measure_datas);
    tracker.RearrangeTracks();
    tracker.TracksPrediction();
    tracker.GetAllObjects(world_obstacles_);
}

void Obstacles::show_obstacles()
{
    if(real_obstacles_.size()==0)
        return;
    cv::Mat img=scanpts_->omni_img_->get_bgr_image().clone();
	size_t numstrans=black_pts_.size();
	for(size_t i=0 ;i<numstrans; i++)
	{
		std::vector<DPoint2i> & pts=black_pts_[i];
		size_t nums_black=pts.size();
		for(size_t j=0 ;j<nums_black;j++)	 
			cv::circle(img,cv::Point(pts[j].x_,pts[j].y_),1,Scalar(0,0,255),2,8,0);
	}

    vector<DPoint2i> img_pts=transfer_->imagecoordunates(real_obstacles_);
    numstrans=img_pts.size();
    for(size_t i=0 ;i<numstrans; i++)
       cv::circle(img,cv::Point(img_pts[i].x_,img_pts[i].y_),2,Scalar(0,255,255),2,8,0);
	imshow("BlackPts",img);
    cv::waitKey(5.0);
}
