#ifndef NUBOT_ROBOT_H_
#define NUBOT_ROBOT_H_

#include "nubot/core/core.hpp"
#include "ros/ros.h"

namespace nubot{

const int THRES_ROBOT_VALID_CONST=15;

class robot
{

public:

    robot(int id=-1,DPoint loc=DPoint(0,0),Angle head=Angle(0),DPoint vec=DPoint(0,0),
               DPoint acc=DPoint(0,0),double w=0.0,double wacc=0.0,
               bool is_kick_off=false,bool is_robot_stuck=false,bool is_robot_slip=false,
               bool isvalid=false,ros::Time time=ros::Time::now(),ros::Duration duration=ros::Duration(60*3,0));

	robot (const robot& _info) ;

	const robot& operator= (const robot& _info)
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
		return *this;
	}

	~robot(void);	
	bool is_valid();
	void set_robot_id(int _id);
    void set_robot_loc(DPoint _loc);
	void set_robot_head(Angle _head);
    void set_robot_vec(DPoint _vec);
    void set_robot_acc(DPoint _acc);
	void set_robot_w(double _w);
	void set_robot_wacc(double _wacc);
	void set_robot_kick(bool _iskick);
	void set_robot_global_localization(bool _isgloabl);
	void set_robot_stuck(bool _isstuck);
	void set_robot_slip(bool _isslip);

    int    get_robot_id();
    DPoint get_robot_loc();
    Angle  get_robot_head();
    DPoint get_robot_vec();
    DPoint get_robot_acc();
	double get_robot_w();
	double get_robot_wacc();
	bool   get_robot_kick();
	bool   get_robot_stuck();
	bool   get_robot_slip();



    void  detect_robot_stuck();

    ros::Duration get_duration();
    void set_duration(ros::Duration duration);
    ros::Time get_time();
    void set_time(ros::Time time);
    void update_duration();
    void update();

private:
	/** @brief the ID of the robot */
    int      robot_id_;
	/** @brief the location of the robot */
    DPoint   robot_loc_;
	/** @brief the head of the robot */
    Angle    robot_head_;
	/** @brief the velocity of the robot */
    DPoint   robot_vec_;
	/** @brief the accelerator of the robot */
    DPoint   robot_acc_;
	/** @brief the velocity of the robot */
	double   robot_w_;
	/** @brief the accelerator of the robot */
	double   robot_wacc_;
	/** @brief Is the robot kick off */
	bool     is_kick_off_;
	/** @brief Is the robot stuck */
	bool     is_robot_stuck_; 
	/** @brief Is the robot slip */
	bool     is_robot_slip_;   
	/** @brief Is the robot slip */
	bool     is_robot_valid_;
	/** @brief   */
    ros::Duration duration_;

    ros::Time time_;
};

}

#endif // ROBOTINFO_H
