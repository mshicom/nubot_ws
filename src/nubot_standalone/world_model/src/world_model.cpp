#include <stdio.h>
#include <signal.h>
#include <time.h>

#include "nubot/world_model/world_model.h"
#include "nubot/world_model/multicast.h"
#include "nubot/rtdb/omni_ball_infor.h"


using namespace nubot;

int
World_Model::initializeRTDB()
{
   if(DB_init() != 0)
       return -1;
   return 0;
}

int
World_Model::releaseRTDB()
{
   DB_free ();
   return 0;
}

void *
nubot::World_Model::receiveDataThread(void *tmpThis)
{
    nubot::World_Model* PThis = (nubot::World_Model*)tmpThis;
    int end =0;
    while(!end)
    {
        int buffersize=sizeof(MessageFromCoach);
        if(receiveData(PThis->coach_socket_,& PThis->coach2robot_,buffersize)>0)
            ROS_INFO("receive the coach data");
    }

}


nubot::World_Model::World_Model(int argc,char** argv,const char * name)
{

    if((coach_socket_=openSocket("wlan2"))==-1)
         ROS_WARN("The scoket is not openned");
     pthread_attr_init (&thread_attr_);
     pthread_attr_setinheritsched (&thread_attr_, PTHREAD_INHERIT_SCHED);
     if ((pthread_create(&recvThread_, &thread_attr_,receiveDataThread,this)))
     {
         ROS_INFO("pthread_create");
         closeSocket(coach_socket_);
     }

   ros::init(argc,argv,name);
   /** initialize the RTDB database*/
  // initializeRTDB();

   nh = boost::make_shared<ros::NodeHandle>();
   lh = boost::make_shared<ros::NodeHandle>();
   th = boost::make_shared<ros::NodeHandle>();

   //robotinfo_pub_ = lh->advertise<nubot_standalone::RobotInfo>("/worldmodel/robotinfo",10);
   //ballinfo_pub_  = lh->advertise<nubot_standalone::BallInfo>("/worldmodel/ballinfo",10);
   worldmodelinfo_pub_ = lh->advertise<nubot_standalone::WorldModelInfo>("/worldmodel/worldmodelinfo",10);

   omin_vision_sub_= nh->subscribe("/omnivision/OminiVisionInfo", 1 , &nubot::World_Model::updateOminivision, this);

   worldmodel_update_timer_ = th->createTimer(ros::Duration(0.03),&World_Model::update,this);
   ros::spin();
}

nubot::World_Model::~World_Model(){
    closeSocket(coach_socket_);
}

void
nubot::World_Model::updateOminivision(const nubot_standalone::OminiVisionInfo & omni_info)
{
    ros::Time time_update = omni_info.header.stamp;
    robot_info_.set_time(time_update);
    robot_info_.set_robot_vec(DPoint2d(omni_info.robotinfo.vtrans.x,omni_info.robotinfo.vtrans.y));
    robot_info_.set_robot_loc(DPoint2d(omni_info.robotinfo.pos.x,omni_info.robotinfo.pos.y));
    robot_info_.set_robot_w(omni_info.robotinfo.vrot);
    robot_info_.set_robot_head(Angle(omni_info.robotinfo.heading.theta));
    ROS_INFO("x: %f y: %f angle: %f ",omni_info.robotinfo.pos.x,omni_info.robotinfo.pos.y,omni_info.robotinfo.heading.theta);

    std::vector< obstacle_object > obstacles;
    obstacles.reserve(omni_info.obstacleinfo.length );
    for(std::size_t i = 0; i< omni_info.obstacleinfo.length ; i++)
    {
        obstacle_object obs_temp;
        obs_temp.set_obstacle_loc(DPoint2d(omni_info.obstacleinfo.pos[i].x,omni_info.obstacleinfo.pos[i].y));
        obstacles.push_back(obs_temp);
        ROS_INFO("x: %f y: %f",omni_info.obstacleinfo.pos[i].x,omni_info.obstacleinfo.pos[i].y);
    }
    obstacles_.set_omni_obstacles(obstacles);
    obstacles_.set_time(time_update);

    ball_object omni_ball;
    omni_ball.set_ball_global_loc(DPoint(omni_info.ballinfo.pos.x,omni_info.ballinfo.pos.y));
    omni_ball.set_ball_pos_known(omni_info.ballinfo.pos_known);
    omni_ball.set_ball_vec(DPoint(omni_info.ballinfo.velocity.x,omni_info.ballinfo.velocity.y));
    omni_ball.set_ball_real_loc(PPoint(Angle(omni_info.ballinfo.real_pos.angle),omni_info.ballinfo.real_pos.radius));
    omni_ball.set_ball_time(time_update);
    ball_info_.set_omni_ball(omni_ball);
    ROS_INFO("x: %f y: %f vx: %f  vy: %f",omni_info.ballinfo.pos.x,omni_info.ballinfo.pos.y,
             omni_info.ballinfo.velocity.x,omni_info.ballinfo.velocity.x);

}
void
nubot::World_Model::update(const ros::TimerEvent & )
{

    ball_info_.update();
    robot_info_.update();
    for(std::size_t i = 0 ; i < OUR_TEAM ;i ++ )
        teamnate_[i].update();
    obstacles_.update();
    sendtocoach();
    publish();
}

void
nubot::World_Model::publish()
{
   worldmodelinfo_.robotinfo.pos.x  = robot_info_.get_robot_loc().x_;
   worldmodelinfo_.robotinfo.pos.y  = robot_info_.get_robot_loc().y_;
   worldmodelinfo_.robotinfo.heading.theta = robot_info_.get_robot_head().radian_;
   worldmodelinfo_.robotinfo.vtrans.x = robot_info_.get_robot_vec().x_;
   worldmodelinfo_.robotinfo.vtrans.y = robot_info_.get_robot_vec().y_;
   worldmodelinfo_.robotinfo.isstuck  = robot_info_.get_robot_stuck();
   worldmodelinfo_.robotinfo.iskick   = robot_info_.get_robot_kick();
   worldmodelinfo_.robotinfo.isvalid  = robot_info_.is_valid();
   worldmodelinfo_.robotinfo.vrot     = robot_info_.get_robot_w();


   std::vector< obstacle_object > obstacles = obstacles_.get_omni_obstacles();
   worldmodelinfo_.obstacleinfo.length=obstacles.size();
   for(std::size_t i = 0; i< obstacles.size() ; i++)
   {
       worldmodelinfo_.obstacleinfo.pos[i].x= obstacles[i].get_obstacle_loc().x_;
       worldmodelinfo_.obstacleinfo.pos[i].y= obstacles[i].get_obstacle_loc().y_;
   }

   ball_object  mergeball=ball_info_.get_merge_ball();
   worldmodelinfo_.ballinfo.pos.x =  mergeball.get_ball_global_loc().x_;
   worldmodelinfo_.ballinfo.pos.y =  mergeball.get_ball_global_loc().y_;
   worldmodelinfo_.ballinfo.velocity.x = mergeball.get_ball_vec().x_;
   worldmodelinfo_.ballinfo.velocity.y = mergeball.get_ball_vec().y_;
   worldmodelinfo_.ballinfo.velocity_known=mergeball.is_ball_vec_known();
   worldmodelinfo_.ballinfo.pos_known=mergeball.is_ball_pos_known();
   worldmodelinfo_.ballinfo.ballinfostate=ball_info_.get_ball_info_state();

   worldmodelinfo_.coachinfo.game_ctrl=coach2robot_.ctrl;
   worldmodelinfo_.coachinfo.indist=coach2robot_.dist;
   worldmodelinfo_.coachinfo.target.x=coach2robot_.target.x;
   worldmodelinfo_.coachinfo.target.y=coach2robot_.target.y;

   worldmodelinfo_pub_.publish(worldmodelinfo_);

   /*int value;
   int lifetime = DB_get(Agent0, VALUE, &value);
   printf("Value from Agent 0 = %d\t\tdata age = %dms\n", value, lifetime);
   lifetime = DB_get(Agent1, VALUE, &value);
   printf("Value from Agent 1 = %d\t\tdata age = %dms\n", value, lifetime);
   lifetime = DB_get(Agent2, VALUE, &value);
   printf("Value from Agent 2 = %d\t\tdata age = %dms\n", value, lifetime);

   OmniBallInfor omni_ball_infor;
   char buffer[2];

   lifetime = DB_get(Agent0, OMNI_BALL_INFOR, &omni_ball_infor);
//    printf("Ball estimation from Agent 0: pos(%d, %d), vel(%d, %d), acc(%d, %d); \tdata age = %dms\n",
//           omni_ball_infor.px, omni_ball_infor.py,
//           omni_ball_infor.vx, omni_ball_infor.vy,
//           omni_ball_infor.ax, omni_ball_infor.ay, lifetime);
   std::cout << "px: " << static_cast<int>(omni_ball_infor.px[1] * 128 + omni_ball_infor.px[0]) << "; "
             << "py: " << static_cast<int>(omni_ball_infor.py[1] * 128 + omni_ball_infor.py[0]) << std::endl;

   lifetime = DB_get(Agent1, OMNI_BALL_INFOR, &omni_ball_infor);
//    printf("Ball estimation from Agent 1: pos(%d, %d), vel(%d, %d), acc(%d, %d); \tdata age = %dms\n",
//           omni_ball_infor.px, omni_ball_infor.py,
//           omni_ball_infor.vx, omni_ball_infor.vy,
//           omni_ball_infor.ax, omni_ball_infor.ay, lifetime);
   std::cout << "px: " << static_cast<int>(omni_ball_infor.px[1] * 128) + static_cast<int>(omni_ball_infor.px[0]) << "; "
             << "py: " << static_cast<int>(omni_ball_infor.py[1] * 128) + static_cast<int>(omni_ball_infor.py[0]) << std::endl;

   lifetime = DB_get(Agent2, OMNI_BALL_INFOR, &omni_ball_infor);
//    printf("Ball estimation from Agent 2: pos(%d, %d), vel(%d, %d), acc(%d, %d); \tdata age = %dms\n",
//           omni_ball_infor.px, omni_ball_infor.py,
//           omni_ball_infor.vx, omni_ball_infor.vy,
//           omni_ball_infor.ax, omni_ball_infor.ay, lifetime);
   std::cout << "px: " << static_cast<int>(omni_ball_infor.px[1] * 128) + static_cast<int>(omni_ball_infor.px[0]) << "; "
             << "py: " << static_cast<int>(omni_ball_infor.py[1] * 128) + static_cast<int>(omni_ball_infor.py[0]) << std::endl;
*/
}

void
nubot::World_Model::receivefromcoach()
{

}

void
nubot::World_Model::receivefromteamnates()
{


}

void
nubot::World_Model::sendtocoach()
{
    robot2coach_.robot_id=2;

    robot2coach_.ball_global_loc.x=ball_info_.merge_ball_.get_ball_global_loc().x_;
    robot2coach_.ball_global_loc.y=ball_info_.merge_ball_.get_ball_global_loc().y_;

    DPoint ball_real(ball_info_.merge_ball_.get_ball_real_loc());
    robot2coach_.ball_real_loc.x=ball_real.x_;
    robot2coach_.ball_real_loc.y=ball_real.y_;
    robot2coach_.robot_head=robot_info_.get_robot_head().radian_;
    robot2coach_.robot_global_loc.x=robot_info_.get_robot_loc().x_;
    robot2coach_.robot_global_loc.y=robot_info_.get_robot_loc().y_;
    robot2coach_.obs_cnt=obstacles_.get_nums_obstacles();
   // for(std::size_t i = 0; i< robot2coach_.obs_cnt; i++)
   // {
   //     std::vector< nubot::obstacle_object > obstacles = obstacles_.get_omni_obstacles();
   //     robot2coach_.otherRobot[i].x = obstacles[i].get_obstacle_loc().x_;
   //     robot2coach_.otherRobot[i].y = obstacles[i].get_obstacle_loc().y_;
   // }
    robot2coach_.is_kick_off=false;
    robot2coach_.net_state=false;
    robot2coach_.is_dribble=false;
    robot2coach_.ball_info_state=ball_info_.get_ball_info_state();
    robot2coach_.role=0;
    robot2coach_.strategy=0;
    robot2coach_.kick_target.x=0;
    robot2coach_.kick_target.y=0;

    int buffer=sizeof(MessageToCoach);
    if (sendData(coach_socket_,&robot2coach_,buffer) != buffer)
       printf("this data is error");
}
void
nubot::World_Model::sendtoteamnates(){;}
