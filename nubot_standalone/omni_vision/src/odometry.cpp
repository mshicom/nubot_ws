#include "nubot/omni_vision/odometry.h"

using namespace nubot;
Odometry::Odometry(void)
{
	delta_angle_       = Angle(0);
    delta_real_pos_    = DPoint(0,0);
    delta_world_pos_   = DPoint(0,0);
	angle_             = Angle(0);
    world_velocity_    = DPoint(0,0);
    real_velocity_     = DPoint(0,0);
    angular_velocity_  = 0.0;
}
void 
Odometry::clear(const Angle & _angle)
{
	delta_angle_     = Angle(0);
    delta_real_pos_  = DPoint(0,0);
    delta_world_pos_ = DPoint(0,0);
	angle_           = _angle;
}
double
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
Odometry::process(std::vector<int> & _motor_data,double duration)
{
	int length=_motor_data.size();
	if(length!=MOTOR_NUMS_CONST)
		return false;
	static std::vector<float>  tempdata(MOTOR_NUMS_CONST,0);
    DPoint interval_world_pos,interval_real_pos;
    Angle  interval_angle;
    DPoint tmp_world_velocity,tmp_real_velocity;
    double tmp_angular_velocity;

	for(int j=0; j < MOTOR_NUMS_CONST; j++) 
        tempdata[j]=(float)_motor_data[j];

    float rate = RATE;
    float chassisradius =  CHASSISRADIUS;

    tmp_angular_velocity= ((-tempdata[1] - tempdata[3])/(2*chassisradius*rate));
    tmp_real_velocity.x_= ((0.707*(tempdata[0]+tempdata[1]-tempdata[2]-tempdata[3]))/(2*rate));
    tmp_real_velocity.y_= ((0.707*(tempdata[1]+tempdata[2]-tempdata[0]-tempdata[3]))/(2*rate));
    tmp_world_velocity  = tmp_real_velocity.rotate(-angle_);

    interval_world_pos  = tmp_world_velocity * duration;
    interval_real_pos   = tmp_real_velocity  * duration;
    interval_angle      = Angle(tmp_angular_velocity * duration);

	delta_real_pos_ += interval_real_pos;
    delta_world_pos_+= DPoint(interval_world_pos.x_,interval_world_pos.y_);
	delta_angle_    += interval_angle;
	angle_          += interval_angle;

    world_velocity_   =  tmp_world_velocity;
    real_velocity_    =  tmp_real_velocity;
    angular_velocity_ =  tmp_angular_velocity;
	return true;
}

