#include "nubot/world_model/ball.h"
#include <float.h>

nubot::ball_object::ball_object(int id,bool is_valid,DPoint loc ,PPoint real_loc,
                                          DPoint velocity, DPoint acc , ros::Time time,
                                          bool pos_known, bool vec_known,ros::Duration duration)
{
    robot_id_        = id;
    ball_global_loc_ = loc;
    ball_real_loc_   = real_loc;
    ball_acc_        = acc;
    is_ball_valid_   = is_valid;
    ball_velocity_   = velocity;
    ball_pos_known_  = pos_known;
    ball_vec_known_  = vec_known;
    ball_time_       = time;
    duration_        = duration;
}
nubot::ball_object::ball_object (const ball_object& _info) 
{
    robot_id_        = _info.robot_id_;
    ball_global_loc_ = _info.ball_global_loc_;
    ball_real_loc_   = _info.ball_real_loc_;
    ball_acc_        = _info.ball_acc_;
    is_ball_valid_   = _info.is_ball_valid_;
    ball_velocity_   = _info.ball_velocity_;
    ball_pos_known_  = _info.ball_pos_known_;
    ball_vec_known_  = _info.ball_vec_known_;
    ball_time_       = _info.ball_time_;
    duration_        = _info.duration_;
}



void nubot::ball_object::set_ball_id(int _id)                { robot_id_=_id; }
void nubot::ball_object::set_ball_valid(bool is_valid)       { is_ball_valid_=is_valid; }
void nubot::ball_object::set_ball_global_loc(DPoint _loc)    { ball_global_loc_=_loc; }
void nubot::ball_object::set_ball_real_loc(PPoint _loc)      { ball_real_loc_=_loc; }
void nubot::ball_object::set_ball_vec(DPoint _vec)           { ball_velocity_=_vec; }
void nubot::ball_object::set_ball_acc(DPoint _acc)           { ball_acc_=_acc; }
void nubot::ball_object::set_ball_time(const ros::Time time) { ball_time_=time; }
void nubot::ball_object::set_ball_pos_known(bool pos_known)  { ball_pos_known_=pos_known; }
void nubot::ball_object::set_ball_vec_known(bool vec_known)  { ball_pos_known_=vec_known; }
void nubot::ball_object::set_duration(ros::Duration duration){ duration_=duration;}

int           nubot::ball_object::get_ball_id()         { return robot_id_;}
nubot::DPoint nubot::ball_object::get_ball_global_loc() { return ball_global_loc_;}
nubot::PPoint nubot::ball_object::get_ball_real_loc()   { return ball_real_loc_;}
nubot::DPoint nubot::ball_object::get_ball_vec()        { return ball_velocity_;}
nubot::DPoint nubot::ball_object::get_ball_acc()        { return ball_acc_;}
ros::Time     nubot::ball_object::get_ball_time()       { return ball_time_; }
ros::Duration nubot::ball_object::get_duration()        { return duration_;}


bool nubot::ball_object::is_valid()                     { return is_ball_valid_;}
bool nubot::ball_object::is_ball_pos_known()            { return ball_pos_known_;}
bool nubot::ball_object::is_ball_vec_known()            { return ball_vec_known_;}

//! the duration between now and the time when the ball was detected
void nubot::ball_object::update_duration()
{
    ros::Time time=ros::Time::now();
    duration_=time-ball_time_;
}


nubot::ball::ball(void)
{

}

nubot::ball::~ball(void)
{


}

void
nubot::ball::update_duration()
{
    omni_ball_.update_duration();
	for(std::size_t i = 0 ;i < OUR_TEAM ;i++)
        TM_ball_[i].update_duration();
}
void
nubot::ball::update()
{
    update_duration();
    merge_ball();


}
void
nubot::ball::merge_ball()
{
  // the ball is detected by the omni_vision;
  if(omni_ball_.get_duration().toSec()<0.03*2 && omni_ball_.is_ball_pos_known())
  {
     merge_ball_= omni_ball_;
     ball_info_state_ = SEEBALLBYOWN;
  }
  else
  {
      for(std::size_t i =0 ;i < OUR_TEAM ; i++)
      {
          double Min_dis=DBL_MAX;
          std::size_t label=-1;
          if(TM_ball_[i].get_duration().toSec()<0.03*2 && TM_ball_[i].is_ball_pos_known())
          {

              if(Min_dis > TM_ball_[i].get_ball_real_loc().radius_)
               {
                  Min_dis=TM_ball_[i].get_ball_real_loc().radius_;
                  label=i;
               }
          }
          if(label!=-1)
          {
              merge_ball_ = TM_ball_[label];
              ball_info_state_=SEEBALLBYOTHERS ;
          }
          else
          {
              merge_ball_.set_ball_pos_known(false);
              ball_info_state_= NOTSEEBALL ;
          }
      }
   }
}

void nubot::ball::record_ball_information()
{

}

int
nubot::ball::get_ball_info_state()                        { return ball_info_state_;}
void
nubot::ball::set_ball_info_state(int _state)             { ball_info_state_ = _state;}

nubot::ball_object
nubot::ball::get_merge_ball()                             { return merge_ball_;}

void
nubot::ball::set_merge_ball(nubot::ball_object ball_obg)  { merge_ball_ = ball_obg;}

nubot::ball_object
nubot::ball::get_omni_ball()                              { return omni_ball_;}

void
nubot::ball::set_omni_ball(nubot::ball_object ball_obg)   { omni_ball_ = ball_obg;}


bool
nubot::ball::set_TM_ball(nubot::ball_object ball_obg)
{
	if(ball_obg.get_ball_id()!=-1)
	{
		TM_ball_[ball_obg.get_ball_id()]=ball_obg;
		return true;
	}
	else
		return false;
}
