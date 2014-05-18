#include "nubot/world_model/robot.h"

nubot::robot::robot(int id,DPoint loc,Angle head,DPoint vec,
                            DPoint acc,double w,double wacc,
							bool  is_kick_off,bool is_robot_stuck,bool is_robot_slip,
                            bool isvalid,ros::Time time,ros::Duration duration)
{
	robot_id_  = id;
	robot_loc_ = loc;
	robot_head_= head;
	robot_vec_ = vec;
	robot_acc_ = acc;
	robot_w_   = w;
	robot_wacc_= wacc;
    time_      = time;
    duration_  = duration;
	is_kick_off_   = is_kick_off;
	is_robot_stuck_ = is_robot_stuck; 
	is_robot_slip_  = is_robot_slip;  
	is_robot_valid_ = isvalid;
}

nubot::robot::robot(const robot&_info)
{
    robot_id_   = _info.robot_id_;
    robot_loc_  = _info.robot_loc_;
    robot_head_ = _info.robot_head_;
    robot_vec_  = _info.robot_vec_;
    robot_w_    = _info.robot_w_;
    robot_wacc_ = _info.robot_wacc_;
    robot_acc_  = _info.robot_acc_;
    is_kick_off_= _info.is_kick_off_;
    time_       = _info.time_;
    duration_   = _info.duration_;
    is_robot_valid_ = _info.is_robot_valid_;
    is_robot_stuck_ = _info.is_robot_stuck_;;
    is_robot_slip_  = _info.is_robot_slip_;;
}
nubot::robot::~robot(){ }

void nubot::robot::set_robot_id(int _id)                { robot_id_=_id;  }
void nubot::robot::set_robot_loc(DPoint _loc)           { robot_loc_=_loc; }
void nubot::robot::set_robot_head(Angle _head)          { robot_head_=_head;}
void nubot::robot::set_robot_vec(DPoint _vec)           { robot_vec_=_vec;}
void nubot::robot::set_robot_acc(DPoint _acc)           { robot_acc_=_acc;}
void nubot::robot::set_robot_w(double _w)               { robot_w_=_w;}
void nubot::robot::set_robot_wacc(double _wacc)         { robot_wacc_=_wacc;}
void nubot::robot::set_robot_kick(bool _iskick)         { is_kick_off_=_iskick;}
void nubot::robot::set_robot_slip(bool _isslip)         { is_robot_slip_=_isslip;}
void nubot::robot::set_robot_stuck(bool _isstuck)       { is_robot_stuck_=_isstuck;}
void nubot::robot::set_time(ros::Time time)             { time_=time;}
void nubot::robot::set_duration(ros::Duration duration) {duration_=duration;}


int  nubot::robot::get_robot_id()           { return robot_id_;}
nubot::DPoint nubot::robot::get_robot_loc() { return robot_loc_;}
nubot::Angle nubot::robot::get_robot_head() { return robot_head_;}
nubot::DPoint nubot::robot::get_robot_vec() { return robot_vec_;}
nubot::DPoint nubot::robot::get_robot_acc() { return robot_acc_;}
double nubot::robot::get_robot_w()          { return robot_w_;}
double nubot::robot::get_robot_wacc()       { return robot_wacc_;}
bool   nubot::robot::get_robot_kick()       { return is_kick_off_;}
bool   nubot::robot::get_robot_stuck()      { return is_robot_stuck_;}
bool   nubot::robot::get_robot_slip()       { return is_robot_slip_;}
ros::Duration nubot::robot::get_duration()  { return duration_;}
ros::Time nubot::robot::get_time()          { return time_;}

void
nubot::robot::update_duration()
{ 
     ros::Time time=ros::Time::now();
     duration_ = time_-time;
}
bool   nubot::robot::is_valid()             { return is_robot_valid_;}

void
nubot::robot::detect_robot_stuck()
{

}

void nubot::robot::update()
{
    update_duration();
    if(duration_.toSec()>0.03*10)
       is_robot_valid_=false;
    else
        is_robot_valid_=true;
}

