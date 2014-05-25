#include "nubot/omni_vision/ballfinder.h"
#include "ros/ros.h"

using namespace nubot;

BallFinder::BallFinder(Transfer & _transfer)
{
  transfer_ = &_transfer;
  ball_record_.reserve(15);
  ball_time_.reserve(15);
  record_velocity_.reserve(5);
}

BallFinder::~BallFinder()
{

}

bool
BallFinder::Process(cv::Mat &_image,const DPoint & _location, const Angle & _angle )
{
  static DPoint robot_last_loc=_location;
  static Angle  robot_angle=_angle;
  static ros::Time time_before=ros::Time::now();
  ball_vec_known_  = false;

  bool is_detected_ball=false;
  if(_image.cols==0 || _image.rows==0)
     return false;
  if(_image.cols!=check_flag_.cols || _image.rows!=check_flag_.rows)
       check_flag_.create(_image.rows,_image.cols,CV_8UC1);

  is_detected_ball=RegionSearch(_image,ball_area_);

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

bool
nubot::BallFinder::RegionSearch(cv::Mat & _segment_img,std::vector<ImageArea> &_target_areas, const int &_max_num_of_areas,
                                const cv::Rect &_ROI, const int &_threshold_size, const int &_threshold_combination,
                                unsigned char _target_color)
{
  if(check_flag_.cols!=_segment_img.cols || check_flag_.rows!=_segment_img.rows)
    return(false);
  if(_segment_img.rows*_segment_img.cols<_threshold_size)
    return(false);
  if((double)(_ROI.width)*_ROI.height<_threshold_size)
    return(false);

  //some initializations
  static const cv::Point2i neighbors[8]={cv::Point(-1,-1),cv::Point(0,-1),cv::Point(1,-1),
                                         cv::Point(-1,0),                 cv::Point(1, 0),
                                         cv::Point(-1, 1),cv::Point(0, 1),cv::Point(1, 1)};
  static ImageArea current_area;//the current area
  static long x_sum, y_sum;//used to calculate the centroid of current area
  static int area_rect_Xmin;//the ex-rectangle of current area
  static int area_rect_Ymin;//the ex-rectangle of current area
  static int area_rect_Xmax;//the ex-rectangle of current area
  static int area_rect_Ymax;//the ex-rectangle of current area
  static cv::Point current_point;//the current point when processing
  static cv::Point neighbor_point;//the neighbor of current point when region grow
  static int neighbor_amount;//count the neighbors of current point. if less than 8, current point is edge point

  check_flag_.setTo(0);
  int Xmin = _ROI.x;//can be equal
  int Ymin = _ROI.y;//can be equal
  int Xmax = _ROI.x+_ROI.width;//can NOT be equal
  int Ymax = _ROI.y+_ROI.height;//can NOT be equal
  Xmin = cv::max(0,cv::min(_segment_img.cols,Xmin));
  Xmax = cv::max(0,cv::min(_segment_img.cols,Xmax));
  Ymin = cv::max(0,cv::min(_segment_img.rows,Ymin));
  Ymax = cv::max(0,cv::min(_segment_img.rows,Ymax));
  std::vector<ImageArea> area_list;//once a candidate area is found, it will be pushed in this list

  //start process
  //1. search areas no mater how small they are
  for(int y=Ymin; y<Ymax; y++)
  {
    for(int x=Xmin; x<Xmax; x++)
    {
      current_point = cv::Point(x,y);
      //for every pixel, if it is _target_color, it must be put in an area. the check_flag_ mat marked if a pixel has already been put in a area.
      if(_segment_img.at<unsigned char>(current_point)==_target_color && !check_flag_.at<unsigned char>(current_point))
      {
        //if a pixel is _target_color, but has not been put in an area, a new area is established to take this pixel in.  then find all pixels which is connected with this area and put them in.
        check_flag_.at<unsigned char>(current_point) = true;
        grow_queue_.push_back(current_point);
        x_sum = 0;
        y_sum = 0;
        current_area.edge_points_.clear();
        current_area.area_size_ = 0;
        area_rect_Xmin = area_rect_Xmax = current_point.x;
        area_rect_Ymin = area_rect_Ymax = current_point.y;
        //start to search pixels which is connected with this area and put them in
        while(grow_queue_.size()>0)
        {
          current_point = grow_queue_.front();
          x_sum += current_point.x;
          y_sum += current_point.y;
          current_area.area_size_++;
          neighbor_amount = 0;
          for(int k=0; k<8; k++)
          {
            neighbor_point = current_point+neighbors[k];
            if(neighbor_point.x<Xmax && neighbor_point.x>=Xmin && neighbor_point.y<Ymax && neighbor_point.y>Ymin && _segment_img.at<unsigned char>(neighbor_point)==_target_color)
            {
              neighbor_amount++;
              if(check_flag_.at<unsigned char>(neighbor_point)==false)
              {
                check_flag_.at<unsigned char>(neighbor_point) = true;
                grow_queue_.push_back(neighbor_point);
              }
            }
          }
          if(neighbor_amount<8)
          {
            //storage edge points of current area
            current_area.edge_points_.push_back(current_point);
            //update area rect of current area
            if(current_point.x<area_rect_Xmin)
              area_rect_Xmin = current_point.x;
            else if(current_point.x>area_rect_Xmax)
              area_rect_Xmax = current_point.x;
            if(current_point.y<area_rect_Ymin)
              area_rect_Ymin = current_point.y;
            else if(current_point.y>area_rect_Ymax)
              area_rect_Ymax = current_point.y;
          }
          grow_queue_.pop_front();
        }
        //one area is complete
        if(current_area.area_size_>=_threshold_size)
        {
          current_area.area_center_ = cv::Point(x_sum/current_area.area_size_,y_sum/current_area.area_size_);
          current_area.area_rect_ = cv::Rect(area_rect_Xmin,area_rect_Ymin,area_rect_Xmax-area_rect_Xmin,area_rect_Ymax-area_rect_Ymin);
          area_list.push_back(current_area);
        }

      }//end of processing one area
    }//for(int x=Xmin; x<Xmax; x++)
  }//for(int y=Ymin; y<Ymax; y++)

  if(area_list.size()==0)
    return(false);

  //2. combine areas nearby
  if(_threshold_combination>0)
  {

  }

  //3. rank the areas by size
  std::vector<ImageArea>::iterator temp_area_iterator;
  std::vector<ImageArea>::iterator current_area_iterator;
  _target_areas.clear();
  for(int i=0; i<_max_num_of_areas; i++)
  {
    if(area_list.size()==0)
      break;
    //search the max size area in area_list and put it in _target_areas.
    current_area_iterator = area_list.begin();
    for(temp_area_iterator=area_list.begin(); temp_area_iterator!=area_list.end(); temp_area_iterator++)
    {
      if(temp_area_iterator->area_size_>current_area_iterator->area_size_)
        current_area_iterator = temp_area_iterator;
    }
    _target_areas.push_back(*current_area_iterator);
    area_list.erase(current_area_iterator);
  }
  return(true);
}

bool
nubot::BallFinder::RegionSearch(cv::Mat & _segment_img,ImageArea &_target_area)
{
  std::vector<ImageArea> areas;
  if(!RegionSearch(_segment_img,areas))
    return(false);
  _target_area = *areas.begin();
  return(true);
}

DPoint
BallFinder::get_ball_global_loc()  {return ball_global_loc_;}
PPoint
BallFinder::get_ball_real_loc()    {return ball_real_loc_;}
DPoint
BallFinder::get_ball_velocity()    {return ball_global_vec_;}
bool
BallFinder::is_velocity_known()    {return ball_vec_known_;}
