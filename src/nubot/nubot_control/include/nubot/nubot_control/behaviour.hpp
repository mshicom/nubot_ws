#ifndef BEHAVIOUR_H
#define BEHAVIOUR_H

#include <iostream>
#include <stdio.h>
#include <cmath>
#include <string.h>
#include"nubot/core/core.hpp"
#include "common.hpp"
//#include"nubot/nubot_control/define.hpp"

using namespace std;
namespace nubot{

   class WolrdModeliInfo
   {
       public:
              WolrdModeliInfo()
                  :active_robot_num_(0),is_robot_stuck_(false),ball_info_state_(0),
                    game_ctrl_(CTRL_STOP),indist_(0)
              {
                  robot_pos_ = DPoint(0.0,0.0);
                  robot_ori_ = Angle(0.0);
                  robot_vel_ = DPoint(0.0,0.0);
                  ball_pos_  = DPoint(0.0,0.0);
                  ball_vel_  = DPoint(0.0,0.0);
                  obs_pos_.reserve(10);
                  target_    = DPoint(0.0,0.0);
                  target_orientation_ = 0.0;
              }
              ~WolrdModeliInfo()
              {}
       public:

              int    active_robot_num_;   //current active robot number

              DPoint robot_pos_;          //robot position
              Angle  robot_ori_;          //robot orientation
              DPoint robot_vel_;          //robot velocity
              bool   is_robot_stuck_;     //stuck check

              DPoint ball_pos_;           //ball position
              DPoint ball_vel_;           //ball velocity
              int    ball_info_state_;    //ball information state    local ?  share ? or cannot see


              std::vector<DPoint> obs_pos_;            //obstacle information

              unsigned char game_ctrl_;   //current order

              double    indist_;
              DPoint    target_;             //target from coach
              float     target_orientation_; //the orientation of the specified target


   };
    class Behaviour
    {public:

               float app_vx_;
               float app_vy_;
               float app_w_;

               WolrdModeliInfo worldmodelinfo_;

        public:
            Behaviour();
            public: // basic control methods
            float
                basicPDControl(float pgain,float dgain, float err,float err1, float maxval);
            float
                basicPIDcontrol(float pgain,
                               float igain,
                               float dgain,
                               float currval,
                               float targetval,
                               float imaxlimiter,
                               float iminlimiter,
                               float& dstate,
                               float& istate );
    // omnidirectional mobile robot control model

    // activedriible control device control model

    // dribble control   //basic control methods

    // tranlation control
            void
                move2Position(float pval, float dval, DPoint target, float maxvel);  // move to the target point by using PD control
            void
                move2target(float pval, float dval,DPoint target, DPoint target_vel, float maxvel);

            void
                bangbangSolution();
            void
                bangbangmove2Position(DPoint target);
            void
                traceTarget();

    // rotation control
            void
                revDecoupleFromVel(float vx,float vy,float &positivemax_rev,float &negativemax_rev);  //!! seems cannot satisfy all the functions ...
            void
                rotate2AbsOrienation(float pval, float dval, float orientation,float maxw);  // rotate to the target orientation by using PD control
            void
                rotate2RelOrienation(float pval, float dval, float rel_orientation,float maxw);
            void
                rotatetowardsSetPoint(DPoint point);
            void
                rotatetowardsRelPoint(DPoint rel_point);

    // accelaration process
            void
            accelerateProcess(DPoint &vr ,double acc,double threshhold);


    // common

            void clearBehaviorState();


     };
}

#endif // BEHAVIOUR_H
