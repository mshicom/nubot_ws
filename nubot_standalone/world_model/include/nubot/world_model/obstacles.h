#ifndef NUBOT_OBSTACLES_H_
#define NUBOT_OBSTACLES_H_

#include "nubot/core/core.hpp"
#include "ros/ros.h"
namespace nubot {

class obstacle_object{

public:

    obstacle_object(DPoint loc=DPoint(10000,10000),DPoint vec=DPoint(0,0),DPoint acc=DPoint(0,0));
	obstacle_object(const obstacle_object & _info);

	const obstacle_object & operator=(const obstacle_object & _info)
	{
		obstacle_vec_ = _info.obstacle_vec_;
		obstacle_loc_ = _info.obstacle_loc_;
		obstacle_acc_ = _info.obstacle_acc_;
		return *this;
	}
    void set_obstacle_loc(DPoint _loc);
    void set_obstacle_vec(DPoint _vec);
    void set_obstacle_acc(DPoint _acc);
    DPoint get_obstacle_loc();
    DPoint get_obstacle_vec();
    DPoint get_obstacle_acc();
private:

    DPoint obstacle_loc_;
    DPoint obstacle_vec_;
    DPoint obstacle_acc_;


};

class obstacles{

public:
	obstacles(void);
	~obstacles(void);

    void set_omni_obstacles(std::vector<obstacle_object> & _obstacles);
	void clear_obstacles();
	std::vector< obstacle_object > get_omni_obstacles();
    int  get_nums_obstacles();

    void record_obstacles();

    std::vector< vector< obstacle_object > > get_record_obstacles();
    std::vector<int>        get_record_nums();
    std::vector<ros::Time>  get_record_time();

    void update();
    void update_duration();

    void set_duration(ros::Duration duration);
    void set_time(ros::Time time);
    ros::Time get_time();
    ros::Duration get_duration();


private:
	std::vector< obstacle_object > obstacles_;
	int nums_obstacles_;
	std::vector< vector< obstacle_object > > record_obstacles_;
    std::vector<int>           record_nums_;
    std::vector<ros::Time>     record_time_;
    ros::Time     time_;
    ros::Duration duration_;
};

}

#endif // OBSTACLEINFO_H
