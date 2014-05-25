#include "nubot/world_model/obstacles.h"

nubot::obstacle_object::obstacle_object(DPoint loc,DPoint vec,DPoint acc)
{
    obstacle_acc_=acc;
	obstacle_vec_=vec;
    obstacle_loc_=loc;
}
nubot::obstacle_object::obstacle_object(const obstacle_object & _info)
{
    obstacle_vec_ = _info.obstacle_vec_;
    obstacle_loc_ = _info.obstacle_loc_;
    obstacle_acc_ = _info.obstacle_acc_;
}
void nubot::obstacle_object::set_obstacle_loc(DPoint _loc) {obstacle_loc_=_loc;}
void nubot::obstacle_object::set_obstacle_vec(DPoint _vec) {obstacle_vec_=_vec;}
void nubot::obstacle_object::set_obstacle_acc(DPoint _acc) {obstacle_acc_=_acc;}
nubot::DPoint nubot::obstacle_object::get_obstacle_loc() { return obstacle_loc_;}
nubot::DPoint nubot::obstacle_object::get_obstacle_vec() { return obstacle_vec_;}
nubot::DPoint nubot::obstacle_object::get_obstacle_acc() { return obstacle_acc_;}


nubot::obstacles::obstacles(void)
{
    time_=ros::Time::now();
    duration_=ros::Duration(60*3,0);
    obstacles_.clear();
    nums_obstacles_=0;;
	record_obstacles_.clear();
	record_nums_.clear();
    record_time_.clear();
}

nubot::obstacles::~obstacles(void)
{

}

void nubot::obstacles::set_omni_obstacles(std::vector< nubot::obstacle_object > & _obstacles)
{
    record_obstacles();
    clear_obstacles();
    nums_obstacles_= _obstacles.size();
    for(std::size_t i =0 ;i < nums_obstacles_;i++)
        obstacles_.push_back(_obstacles[i]);
}
void nubot::obstacles::clear_obstacles()
{
	obstacles_.clear();
	nums_obstacles_=0;;
}

std::vector< nubot::obstacle_object > nubot::obstacles::get_omni_obstacles()
{
	return obstacles_;
}
int nubot::obstacles::get_nums_obstacles()
{
	return nums_obstacles_;
}
void nubot::obstacles::record_obstacles()
{
	if(nums_obstacles_>0)
	{
		record_obstacles_.push_back(obstacles_);
		record_nums_.push_back(nums_obstacles_);
	}
    record_time_.push_back(time_);
}

void nubot::obstacles::set_duration(ros::Duration duration)  { duration_= duration;}
void nubot::obstacles::set_time(ros::Time time)              { time_ = time;}
ros::Time nubot::obstacles::get_time()                       { return time_;}
ros::Duration nubot::obstacles::get_duration()               { return duration_;}

std::vector< std::vector< nubot::obstacle_object > > nubot::obstacles::get_record_obstacles()
{
	return record_obstacles_;
}
std::vector<int> nubot::obstacles::get_record_nums()
{
	return record_nums_;
}
std::vector<ros::Time> nubot::obstacles::get_record_time()
{
    return record_time_;
}

void  nubot::obstacles::update_duration()
{
    ros::Time time=ros::Time::now();
    duration_ =time_ - time;
}
void  nubot::obstacles::update()
{
     update_duration();
}
