#ifndef _NUBOT_World_Model_H_
#define _NUBOT_World_Model_H_

#include "nubot/core/core.hpp"
#include "nubot/world_model/ball.h"
#include "nubot/world_model/robot.h"
#include "nubot/world_model/obstacles.h"
#include <std_msgs/Header.h>
#include <std_msgs/String.h>
#include <nubot_common/BallInfo.h>
#include <nubot_common/RobotInfo.h>
#include <nubot_common/ObstaclesInfo.h>
#include <nubot_common/OminiVisionInfo.h>
#include <nubot_common/WorldModelInfo.h>

//#include "nubot/rtdb/multicast.h"
//#include "nubot/rtdb/rtdb_user.h"
//#include "nubot/rtdb/rtdb_api.h"
#include "ros/ros.h"
#include <semaphore.h>


namespace nubot {


template<typename _T> struct Point_
{
  _T x;
  _T y;
};

typedef Point_<double>  Point2d;

typedef Point2d Point;

struct obs_info_zip
{
    double x,y;
    double HRZ[4];
    double HRH[4];
};

struct MessageFromCoach
{
     int     robot_id;
     unsigned char     ctrl;
     Point   ball_loc;
     int     dist;
     Point   target;
     double  target_angle;
     bool    is_ball_saw;
};

const int MAX_OBSTALCES_NUMS_CONST=9;

struct MessageToCoach
{
    int    robot_id;
    Point  ball_global_loc;
    Point  ball_real_loc;
    Point  robot_global_loc;
    double robot_head;
    unsigned char strategy;
    unsigned char role;
    unsigned char action;
    int  ball_info_state;
    bool is_dribble;
    bool net_state;
    Point otherRobot[MAX_OBSTALCES_NUMS_CONST];
    int obs_cnt;
    obs_info_zip obs_measure[MAX_OBSTALCES_NUMS_CONST];
    bool  is_kick_off;
    Point kick_target;
};


class World_Model
{

public:
    World_Model(int argc,char** argv,const char* name);
    ~World_Model();
    void update(const ros::TimerEvent& event);
    void updateOminivision(const nubot_common::OminiVisionInfo &omniinfo);
    void publish();
    void receivefromcoach();
    void receivefromteamnates();
    void sendtocoach();
    void sendtoteamnates();

    /** Initialize the RTDB database. */
    int
    initializeRTDB();

    /** Release the RTDB database. */
    int
    releaseRTDB();

    static void * receiveDataThread(void *tmpThis);
    static sem_t  coach2robot_pthead_sem_;

public:

    robot     robot_info_;
    robot     teamnate_[OUR_TEAM];
    ball      ball_info_;
    obstacles obstacles_;

    int  coach_socket_;
    MessageToCoach   robot2coach_;
    MessageFromCoach coach2robot_;

    pthread_t recvThread_;
    pthread_attr_t thread_attr_;

public:

    //nubot_standalone::RobotInfo robotinfo;
    //nubot_standalone::BallInfo ballinfo;
    //nubot_standalone::ObstaclesInfo  obstaclesinfo;
    nubot_common::WorldModelInfo worldmodelinfo_;

    //ros::Publisher  robotinfo_pub_;
    //ros::Publisher  ballinfo_pub_;
    ros::Publisher    worldmodelinfo_pub_;

    ros::Subscriber omin_vision_sub_;

    ros::Timer      worldmodel_update_timer_;


    boost::shared_ptr<ros::NodeHandle> nh;
    boost::shared_ptr<ros::NodeHandle> lh;
    boost::shared_ptr<ros::NodeHandle> th;

};

}


#endif // _NUBOT_World_Model_H_
