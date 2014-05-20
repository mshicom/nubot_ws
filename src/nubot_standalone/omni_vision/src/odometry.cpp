﻿#include "nubot/omni_vision/odometry.h"

using namespace nubot;
Odometry::Odometry(void)
{
	delta_angle_       = Angle(0);
    delta_real_pos_    = DPoint(0,0);
    delta_world_pos_   = DPoint(0,0);
	angle_             = Angle(0);
    world_velocity_    = DPoint(0,0);
    real_velocity_     = DPoint(0,0);
	angular_velocity_  = Angle(0);
}
void 
Odometry::clear(const Angle & _angle)
{
	delta_angle_     = Angle(0);
    delta_real_pos_  = DPoint(0,0);
    delta_world_pos_ = DPoint(0,0);
	angle_           = _angle;
}
Angle 
Odometry::get_angular_velocity(){
	return angular_velocity_;}

DPoint
Odometry::get_world_velocity(){
	return world_velocity_;}

DPoint
Odometry::get_real_velocity(){
	return real_velocity_;}

Angle 
Odometry::get_delta_angle(){
	return delta_angle_;}

DPoint
Odometry::get_real_locaton(){
	return delta_real_pos_;}

DPoint
Odometry::get_world_locaton(){
	return delta_world_pos_;}

bool 
Odometry::process(std::vector<double> & _motor_data, double duration)
{
	int length=_motor_data.size();
	if(length!=MOTOR_NUMS_CONST)
		return false;
    static std::vector<double>  tempdata(MOTOR_NUMS_CONST,0);

    DPoint interval_world_pos,interval_real_pos;
	Angle interval_angle;
	for(int j=0; j < MOTOR_NUMS_CONST; j++) 
        tempdata[j]=(float)_motor_data[j];
    real_velocity_    = DPoint(tempdata[0],tempdata[1]);
    angular_velocity_ = tempdata[2];

    interval_real_pos  = real_velocity_ * duration;
    interval_angle     = angular_velocity_* duration;
    world_velocity_    = real_velocity_.rotate(-angle_);
    interval_world_pos = interval_real_pos.rotate(-angle_);

	delta_real_pos_ += interval_real_pos;
    delta_world_pos_+= interval_world_pos;
	delta_angle_    += interval_angle;
	angle_          += interval_angle;

	return true;
}

