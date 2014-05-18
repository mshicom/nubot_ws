#include "nubot/core/core.hpp"


#include "nubot/nubot_control/behaviour.hpp"
#include "nubot/nubot_control/plan.hpp"
#include "nubot/nubot_control/strategy.hpp"

#include <iostream>
#include <stdio.h>
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <std_msgs/String.h>
//#include <nubot_control/Point3d.h>
#include <nubot_standalone/BallInfo.h>
#include <nubot_standalone/RobotInfo.h>
#include <nubot_standalone/ObstaclesInfo.h>
#include <nubot_standalone/VelCmd.h>
#include <nubot_standalone/WorldModelInfo.h>


using namespace std;

namespace nubot{

    class MslControl
    {
    public:

          MslControl();
          ~MslControl();

          void
          loopControl(const ros::TimerEvent& event);
          void
          OmegaDecoupleFromVel(float vx,float vy );
          void
          setEthercatCommond();
          void
          info2coachSend();
     public:

          Strategy m_strategy_;
          DPoint   robot_pos_;
          DPoint   tar_pos_;
          DPoint   ball_pos_;
          Angle    robot_angle_;
          Angle    tar_angle_;
          DPoint   robot_vel_;

          float    positivemax_w_;
          float    negativemax_w_;

          //float Vx;
          //float Vy;
          //float w;



    private:
          //boost::shared_ptr<ros::NodeHandle>  local_nh_;
          //boost::shared_ptr<ros::NodeHandle>  node_nh_;
          //boost::shared_ptr<ros::NodeHandle>  control_nh_;

          ros::Subscriber  ballinfo_sub_;
          ros::Subscriber  robotinfo_sub_;
          ros::Subscriber  obstaclesinfo_sub_;
          ros::Subscriber  worldmodelinfo_sub_;


          ros::Publisher   motor_cmd_pub_;
          ros::Timer       control_timer_;

    private:



          void
          worldimodelinfoUpdate(const nubot_standalone::WorldModelInfo & _worldmodelinfo_msg)
           {
              m_strategy_.robot_pos_ = DPoint(_worldmodelinfo_msg.robotinfo.pos.x,_worldmodelinfo_msg.robotinfo.pos.y);
              m_strategy_.robot_ori_ =  Angle(_worldmodelinfo_msg.robotinfo.heading.theta);
              m_strategy_.robot_vel_ = DPoint(_worldmodelinfo_msg.robotinfo.vtrans.x,_worldmodelinfo_msg.robotinfo.vtrans.y);
              m_strategy_.is_robot_stuck_ = _worldmodelinfo_msg.robotinfo.isstuck;


              m_strategy_.ball_info_state_ =  _worldmodelinfo_msg.ballinfo.ballinfostate;
              m_strategy_.ball_pos_ = DPoint(_worldmodelinfo_msg.ballinfo.pos.x,_worldmodelinfo_msg.ballinfo.pos.y);
              m_strategy_.ball_vel_ = DPoint(_worldmodelinfo_msg.ballinfo.velocity.x,_worldmodelinfo_msg.ballinfo.velocity.y);
              //m_strategy_.obs_pos_

              m_strategy_.game_ctrl_ = _worldmodelinfo_msg.coachinfo.game_ctrl;
              m_strategy_.target_ =  DPoint(_worldmodelinfo_msg.coachinfo.target.x,_worldmodelinfo_msg.coachinfo.target.y);
              m_strategy_.target_orientation_ = _worldmodelinfo_msg.coachinfo.target_orientation;
              m_strategy_.indist_ = _worldmodelinfo_msg.coachinfo.indist;

              for(int i  = 0; i  < 10; i++)
              {
                  m_strategy_.obs_pos_[i].x_ = _worldmodelinfo_msg.obstacleinfo.pos[i].x;
                  m_strategy_.obs_pos_[i].y_ = _worldmodelinfo_msg.obstacleinfo.pos[i].y;
                  m_strategy_.m_plan_.obs_pos_[i] =  m_strategy_.obs_pos_[i];
              }

              m_strategy_.m_plan_.ball_pos_ = m_strategy_.ball_pos_;
              m_strategy_.m_plan_.ball_vel_ = m_strategy_.ball_vel_;

              m_strategy_.m_plan_.robot_pos_ = m_strategy_.robot_pos_;
              m_strategy_.m_plan_.robot_vel_ = m_strategy_.robot_vel_;
              m_strategy_.m_plan_.robot_ori_ = m_strategy_.robot_ori_;

              m_strategy_.m_plan_.m_behaviour_.ball_pos_  =  m_strategy_.m_plan_.ball_pos_;
              m_strategy_.m_plan_.m_behaviour_.ball_vel_  =  m_strategy_.m_plan_.ball_vel_ ;
              m_strategy_.m_plan_.m_behaviour_.robot_pos_ =  m_strategy_.m_plan_.robot_pos_;
              m_strategy_.m_plan_.m_behaviour_.robot_vel_ =  m_strategy_.m_plan_.robot_vel_;
              m_strategy_.m_plan_.m_behaviour_.robot_ori_ =  m_strategy_.m_plan_.robot_ori_;



              return;
           }

    };



   MslControl::MslControl()
   {
     ROS_INFO("initialize control process");

     ros::NodeHandle local_nh;//_ = boost::make_shared<ros::NodeHandle>();
     ros::NodeHandle node_nh;//_ =  boost::make_shared<ros::NodeHandle>();
     ros::NodeHandle control_nh;//_ =   boost::make_shared<ros::NodeHandle>();

     motor_cmd_pub_ = local_nh.advertise<nubot_standalone::VelCmd>("/nubotcotrol/velcmd",1);

    // ballinfo_sub_  =  node_nh.subscribe("/worldmodel/ballinfo", 100, &MslControl::ballinfoUpdate,this);
    // robotinfo_sub_ =  node_nh.subscribe("/worldmodel/robotinfo", 100, &MslControl::robotinfoUpdate,this);
    // obstaclesinfo_sub_ =  node_nh.subscribe("/worldmodel/obstaclesinfo", 100, &MslControl::obstaclesinfoUpdate,this);
     worldmodelinfo_sub_ = node_nh.subscribe("/worldmodel/worldmodelinfo", 100, &MslControl::worldimodelinfoUpdate,this);
     control_timer_ = control_nh.createTimer(ros::Duration(0.03),&MslControl::loopControl,this);

    // robot_pos.x_ = 500;
    // robot_pos.y_ = 100;
    // robot_angle.radian_ = SINGLEPI_CONSTANT/4;

   }
   MslControl::~MslControl()
   {
   }
   void
   MslControl::info2coachSend()
   {

   }
   void
   MslControl::loopControl(const ros::TimerEvent& event)
    {

       DPoint  target1 = DPoint(-650,0);
       DPoint  target2 = DPoint(650,0);
       float   delta_x;
       float   delta_y;

       static int state = 0;

       if(state == 0)
       {
           tar_pos_ = target1;

           if(robot_pos_.distance(tar_pos_) <= 25)
           {
                state = 1;
           }
       }
       else if(state == 1)
       {
           tar_pos_ = target2;

           if(robot_pos_.distance(tar_pos_) <= 25)
           {
                state = 0;
           }
       }

    
       DPoint  l = tar_pos_-robot_pos_;
       tar_angle_.radian_ = l.angle().radian_; //SINGLEPI_CONSTANT/2;//l.angle().radian_;
      

       m_strategy_.m_plan_.m_behaviour_.move2Position(1.25,1.50,tar_pos_,300);
       m_strategy_.m_plan_.m_behaviour_.rotate2AbsOrienation(1.75,1.25,tar_angle_.radian_,10);

       float dis = robot_pos_.distance(tar_pos_);
       float theta = robot_angle_.radian_ - tar_angle_.radian_;
       theta = angularnorm(theta);

       //ROS_INFO("%f,%f,%f,%f,%f",dis,theta,m_strategy_.m_plan_.m_behaviour_.app_w_,m_strategy_.m_plan_.m_behaviour_.app_vx_,m_strategy_.m_plan_.m_behaviour_.app_vy_);
       setEthercatCommond();
       info2coachSend();


       delta_x = m_strategy_.m_plan_.m_behaviour_.app_vx_*0.03;
       delta_y = m_strategy_.m_plan_.m_behaviour_.app_vy_*0.03;
       robot_pos_ = prel2global(robot_pos_,robot_angle_.radian_,DPoint(delta_x,delta_y));
       robot_angle_.radian_ += m_strategy_.m_plan_.m_behaviour_.app_w_*0.03;


       m_strategy_.m_plan_.m_behaviour_.robot_pos_= robot_pos_;
       m_strategy_.m_plan_.robot_pos_ = robot_pos_;
       m_strategy_.robot_pos_ = robot_pos_;

     // robot_angle.radian_ = (double)_robotinfo_msg.heading.theta;

       m_strategy_.m_plan_.m_behaviour_.robot_ori_ = robot_angle_;
       m_strategy_.m_plan_.robot_ori_ = robot_angle_;
       m_strategy_.robot_ori_ = robot_angle_;


     ROS_INFO("%f,%f,%f,%f",robot_pos_.x_ ,robot_pos_.y_, dis,theta);

    }

   void
   MslControl::setEthercatCommond()
   {

       nubot_standalone::VelCmd command;  // 最后是解算出来的结果

       float vx,vy,w;

       vx = m_strategy_.m_plan_.m_behaviour_.app_vx_;
       vy = m_strategy_.m_plan_.m_behaviour_.app_vy_;
       w  = m_strategy_.m_plan_.m_behaviour_.app_w_;


       m_strategy_.m_plan_.m_behaviour_.app_vx_ = 0;
       m_strategy_.m_plan_.m_behaviour_.app_vy_ = 0;
       m_strategy_.m_plan_.m_behaviour_.app_w_  = 0;

       command.Vx = vx;
       command.Vy = vy;
       command.w  = w;

       motor_cmd_pub_.publish(command);

   }
   /* void
    MslControl::setSpeed()
    {
          float rate = GEARREDUCTIONRATIO/20/SINGLEPI_CONSTANT;
          float v[4];

          float vx = m_strategy_.m_plan_.m_behaviour_.app_vx_;
          float vy = m_strategy_.m_plan_.m_behaviour_.app_vy_;
          float w  = m_strategy_.m_plan_.m_behaviour_.app_w_;

          v[0]=(-0.707*(vx+vy)+w*RADIUS)*rate;
          v[1]=( 0.707*vy-0.707*vx+w*RADIUS)*rate;
          v[2]=( 0.707*vx+0.707*vy+w*RADIUS)*rate;
          v[3]=( 0.707*(vx-vy)+w*RADIUS)*rate;

          float vmax=fabs(v[0]);
          int numofmax=0;

          for(int i=1;i<4;i++)
          {
              if(vmax<fabs(v[i]))
              {
                  vmax=fabs(v[i]);
                  numofmax=i;
              }
          }

          if(vmax>126)
              {

                  if (numofmax==0)
                  {
                      if (v[numofmax]>0)
                          vx=1.414*(-126/rate+(-0.707*vy+w*RADIUS));
                      else
                          vx=1.414*(126/rate+(-0.707*vy+w*RADIUS));
                  }
                  else if (numofmax==1)
                  {
                      if (v[numofmax]>0)
                          vx=1.414*(-126/rate+(0.707*vy+w*RADIUS));
                      else
                         vx=1.414*(126/rate+(0.707*vy+w*RADIUS));
                  }
                  else if (numofmax==2)
                  {
                      if (v[numofmax]>0)
                          vx=1.414*(126/rate-(0.707*vy+w*RADIUS));
                      else
                          vx=1.414*(-126/rate-(0.707*vy+w*RADIUS));
                  }
                  else if (numofmax==3)
                  {
                      if (v[numofmax]>0)
                          vx=1.414*(126/rate-(-0.707*vy+w*RADIUS));
                      else
                          vx=1.414*(-126/rate-(-0.707*vy+w*RADIUS));
                  }

                  v[0]=(-0.707*(vx+vy)+w*RADIUS)*rate;
                  v[1]=( 0.707*vy-0.707*vx+w*RADIUS)*rate;
                  v[2]=( 0.707*vx+0.707*vy+w*RADIUS)*rate;
                  v[3]=( 0.707*(vx-vy)+w*RADIUS)*rate;

              }

            vmax=fabs(v[0]);

            for(int i=1;i<4;i++)
           {
              if(vmax<fabs(v[i]))
              {
                  vmax=fabs(v[i]);
              }
           }
            if(vmax>126)
           {
              for(int i=0;i<4;i++)
                  v[i]=v[i]*126/vmax;
           }

            nubot_standalone::MotionCmd  command;

            for(int i=0;i<4;i++)
            {
                v[i]=127.0-v[i];
                if(v[i]>254.0)
                    v[i]=254.0;
                if(v[i]<1)
                    v[i]=1.0;
                command.speed[i+1]=v[i]-0x7f;
            }

            command.speed[0] = 0x00;
            command.speed[5] = 0x00;
            command.speed[6] = 0x00;
            command.speed[7] = 0x00;

           //ROS_INFO("%d,%d,%d,%d",command.speed[1],command.speed[2],command.speed[3],command.speed[4]);
           // command.speed[0] = 0x00;
           // command.speed[1] = 0x8f;
           // command.speed[2] = 0x8f;
           // command.speed[3] = 0x8f;

           // command.speed[4] = 0x8f;
           // command.speed[5] = 0x7f;
           // command.speed[6] = 0x7f;
           // command.speed[7] = 0x00;


            //command.speed[0] = 0x55;
            //command.speed[5] = 0x7f;
            //command.speed[6] = 0x7f;
            //command.speed[7] = 0x00;


            motor_cmd_pub_.publish(command);

           m_strategy_.m_plan_.m_behaviour_.app_vx_ = 0;
           m_strategy_.m_plan_.m_behaviour_.app_vy_ = 0;
           m_strategy_.m_plan_.m_behaviour_.app_w_ = 0;



  }*/


}
using namespace nubot;
int main(int argc, char **argv)
{
     ros::init(argc,argv,"nubot_control_node");
     ROS_INFO("start control process");  // 完成一系列的初始化工作？ 以及相应的报错机制。  只有当所有的传感器信息都已经准备就绪的时候才可以运行
     MslControl nubotcontrol;
     ros::spin();

     return 0;
}
