#ifndef NUBOT_BALL_H_
#define NUBOT_BALL_H_

#include "nubot/core/core.hpp"
#include "ros/ros.h"

#define OUR_TEAM 7

namespace nubot{


const int THRES_BALL_VALID_CONST=15;

enum { NOTSEEBALL = 0, SEEBALLBYOWN = 1,SEEBALLBYOTHERS = 2};

class ball_object{

public:

    ball_object(int id=-1,bool is_valid=false,DPoint loc=DPoint(0,0),PPoint real_loc=PPoint(Angle(0),0.0),
                DPoint velocity=DPoint(0,0),DPoint acc=DPoint(0,0),ros::Time time=ros::Time::now(),
                bool pos_known=false, bool vec_known=false,ros::Duration duration=ros::Duration(60*3,0));
	
	ball_object (const ball_object& _info) ;
	
	const ball_object & operator=(const ball_object & _info)
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
		return *this;
	}

	void set_ball_id(int _id);
	void set_ball_valid(bool is_valid);
    void set_ball_global_loc(DPoint _loc);
    void set_ball_real_loc(PPoint _loc);
    void set_ball_vec(DPoint _vec);
    void set_ball_acc(DPoint _acc);
	void set_ball_pos_known(bool pos_known);
    void set_ball_vec_known(bool vec_known);
    void set_ball_time(ros::Time time);


    int    get_ball_id();
    DPoint get_ball_global_loc();
    PPoint get_ball_real_loc();
    DPoint get_ball_vec();
    DPoint get_ball_acc();

    bool is_valid();
	bool is_ball_pos_known();
    bool is_ball_vec_known();

    ros::Time     get_ball_time();
    ros::Duration get_duration();
    void set_duration(ros::Duration duration);

    void update_duration();

private:

	int       robot_id_;
	bool      is_ball_valid_;

    PPoint    ball_real_loc_;
    DPoint    ball_global_loc_;
    DPoint    ball_velocity_;
    DPoint    ball_acc_;
    bool      ball_pos_known_;
    bool      ball_vec_known_;
    ros::Time      ball_time_;
    ros::Duration  duration_;

};


class ball{

public:
	ball(void);
	~ball(void);

    /** @brief update the duration between now and the time when the ball was detected */
    void update_duration();

    /** @brief update the bal information and merge the ball */
    void update();

    /** @breif merge the ball and get the ball state */
    void merge_ball();

    int get_ball_info_state();

    void set_ball_info_state(int _state=0);

    /** @brief record the fifteen frame ball information*/
	void record_ball_information();



    ball_object get_merge_ball();

    void set_merge_ball(ball_object ball_obg);

    ball_object get_omni_ball();

    void set_omni_ball(ball_object ball_obg);

    bool set_TM_ball(ball_object ball_obg);

public:

    /** @brief the ball information from the omni_vision*/
	ball_object omni_ball_;
	/** @brief the ball information from the teammates  */
	ball_object TM_ball_[OUR_TEAM];
	/** @brief the merged ball information according to our and teammates' ball information */
	ball_object merge_ball_;
	/** @brief variables in this block are related to ball state estimation*/
    int  ball_info_state_;
	/** @brief record the fifteen frame omni_ball_ information*/  
	std::vector<ball_object> ball_record[15];

	Line_     ball_path_ ;
	DPoint2d  ball_dropping_point_;
};

}



#endif
