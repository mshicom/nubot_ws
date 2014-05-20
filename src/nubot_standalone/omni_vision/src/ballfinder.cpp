#include "nubot/omni_vision/ballfinder.h"
#include "ros/ros.h"

using namespace nubot;

BallFinder::BallFinder(const char* _ColorTablePath,Transfer & _transfer)
{
  color_segment_ = new ColorSegment(_ColorTablePath);
  transfer_ = &_transfer;
  ball_record_.reserve(15);
  ball_time_.reserve(15);
  record_velocity_.reserve(5);
}

BallFinder::~BallFinder()
{

}

bool
BallFinder::Process(cv::Mat &_image,const DPoint & _location, const Angle & _angle)
{
  static DPoint robot_last_loc=_location;
  static Angle  robot_angle=_angle;
  static ros::Time time_before=ros::Time::now();
  ball_vec_known_  = false;

  bool is_detected_ball=false;
  if(_image.cols==0 || _image.rows==0)
     return false;
  color_segment_->Segment(_image);
  is_detected_ball=color_segment_->RegionSearch(ball_area_);

  if(is_detected_ball)
  {
      DPoint2i ball_pos(ball_area_.area_center_.x, ball_area_.area_center_.y);
      ball_real_loc_   =transfer_->realcoordinates(ball_pos);
      ball_global_loc_ =transfer_->worldcoordinates(ball_real_loc_,_location,_angle);
      ball_global_vec_ =DPoint(0,0);

      double distance_ball=ball_real_loc_.radius_;
      double distance_robot=_location.distance(robot_last_loc);
      Angle  difference_angle=_angle-robot_angle;

      //! wo have detected the ball, so we can record it
      ball_record_.push_back(ball_global_loc_);
      ros::Duration duration=ros::Time::now()-time_before;
      time_before=ros::Time::now();
      ball_time_.push_back(duration.toSec());//wo should not use the first data;

      if(ball_record_.size()>15)
      {
          ball_record_.erase(ball_record_.begin());
          ball_time_.erase(ball_time_.begin());
      }
      if(evaluate_velocity(distance_ball,distance_robot,difference_angle))
      {
          ball_record_.clear();
          ball_time_.clear();
          record_velocity_.clear();
      }

  }
  else
  {
      //! wo have not detected the ball, so we need clear the ball_record_ and ball_time_
      ball_record_.clear();
      ball_time_.clear();
      record_velocity_.clear();
  }

  robot_last_loc =_location;
  robot_angle   = _angle;
  return is_detected_ball;
}

//! @brief wo can get the ball velocity according to the least square method
bool
BallFinder::evaluate_velocity(double & _distance_ball,double & _distance_robot,Angle & _difference_angle)
{

   if(_distance_ball>600)
       return true;

   int nums_record=ball_record_.size();
   if(nums_record < 4)
       return false;

   std::vector<double> accumulate_time;
   accumulate_time.reserve(nums_record);
   accumulate_time.push_back(0);
   for(std::size_t i = 1 ;i < nums_record ;i++)
       accumulate_time.push_back(accumulate_time[i-1]+ball_time_[i]);

   double lamda(0),sum_t(0),sum_tt(0),sum_x(0),sum_y(0),sum_xt(0),sum_yt(0);

   if(nums_record < 8)
      lamda = 0.05;

   for(std::size_t i =1 ;i < nums_record ; i++)
   {
       sum_t  += accumulate_time[i];
       sum_tt += accumulate_time[i]*accumulate_time[i];
       sum_x  += ball_record_[i].x_;
       sum_y  += ball_record_[i].y_;
       sum_xt += accumulate_time[i]*ball_record_[i].x_;
       sum_yt += accumulate_time[i]*ball_record_[i].y_;
   }

   double fenmu=(nums_record-1)*(lamda+sum_tt)-sum_t*sum_t;
   double p0x(0), p0y(0),tempvx(0),tempvy(0);

   if(fenmu!=0)  //the least square method
   {
        p0x=((lamda+sum_tt)*sum_x-sum_t*sum_xt)/fenmu;
        p0y=((lamda+sum_tt)*sum_y-sum_t*sum_yt)/fenmu;
        tempvx=((nums_record-1)*sum_xt-sum_t*sum_x)/fenmu;
        tempvy=((nums_record-1)*sum_yt-sum_t*sum_y)/fenmu;
    }
    else
    {
        p0x=((lamda+sum_tt)*sum_x-sum_t*sum_xt)/nums_record;
        p0y=((lamda+sum_tt)*sum_y-sum_t*sum_yt)/nums_record;
        tempvx=((nums_record-1)*sum_xt-sum_t*sum_x)/nums_record;
        tempvy=((nums_record-1)*sum_yt-sum_t*sum_y)/nums_record;
    }

    DPoint velocity = DPoint(tempvx,tempvy);

    DPoint ball_pedict=DPoint(p0x+tempvx*accumulate_time[nums_record-1], p0y+tempvy*accumulate_time[nums_record-1]);
    double bias=ball_pedict.distance(ball_record_[nums_record-1]);
    static int nums_predicet_errors=0;
    if(bias > BALL_PREDICT_BIAS_CONST)
        nums_predicet_errors++;
    else
        nums_predicet_errors=0;

    record_velocity_.push_back(velocity);
    if(record_velocity_.size()>5)
        record_velocity_.erase(record_velocity_.begin());
    DPoint accumulate_pt(0,0);
    for(int i=0;i<record_velocity_.size();i++)
        accumulate_pt += record_velocity_[i];
    DPoint average_vec=1.0/double(record_velocity_.size())*accumulate_pt;
    double weight_avgpart=0.3;
    ball_global_vec_=average_vec*weight_avgpart+velocity*(1.0-weight_avgpart);

    int collision_const=5;
    bool is_start_again=false;
    if(nums_predicet_errors >= collision_const) //! start again
    {
         is_start_again=true;
         nums_predicet_errors=0;
    }

    if( _distance_robot> 60 || std::abs(_difference_angle.radian_) > SINGLEPI_CONSTANT/3) //! our robot is too fast
          is_start_again=true;

    //! the velocity is two small, maybe this is noise
    if(ball_global_vec_.norm() < (_distance_ball/20.0+20.0)||is_start_again)
         ball_global_vec_=DPoint(0,0);

    ball_vec_known_ = !is_start_again;  //! start again , this velocity is not known by us

    return is_start_again;

}

DPoint
BallFinder::get_ball_global_loc()  {return ball_global_loc_;}
PPoint
BallFinder::get_ball_real_loc()    {return ball_real_loc_;}
DPoint
BallFinder::get_ball_velocity()    {return ball_global_vec_;}
bool
BallFinder::is_velocity_known()    {return ball_vec_known_;}
