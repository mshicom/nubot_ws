#ifndef BEHAVIOUR_H
#define BEHAVIOUR_H

#include <iostream>
#include <stdio.h>
#include <cmath>
#include"nubot/core/core.hpp"
#include "nubot/core/common.hpp"
//#include"nubot/nubot_control/define.hpp"

using namespace std;
namespace nubot{

    class Behaviour
    {
        public:
               float app_vx_;
               float app_vy_;
               float app_w_;

               float app_leftdribblev_;
               float app_rightdribblev_;

               DPoint robot_pos_;
               Angle  robot_ori_;
               DPoint robot_vel_;
               DPoint ball_pos_;
               DPoint ball_vel_;

               //world_model* m_worldmodel_;

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
