#ifndef PLAN_H
#define PLAN_H

#include <cmath>
#include "nubot/core/core.hpp"
#include "nubot/nubot_control/behaviour.hpp"


using namespace std;
namespace nubot{
    class Plan
    {
    public:
        Plan();
        /*******************catch ball******************/
        void 
        traceBall();      // trace
        void
        interceptBall();  // intercept

        void
        catchBall();

        void 
        catchMovingBall();
        void 
        catchMotionlessBall();

        void
        positionAvoidObs(DPoint target, float theta, float stopdis, float stoptheta);

        DPoint
        avoidRelOble(DPoint target, double theta, double ro, double vel, bool includeball);


        void
        catchBallbyPN();

        
    public:
        Behaviour m_behaviour_;

        DPoint robot_pos_;
        Angle  robot_ori_;
        DPoint robot_vel_;
        DPoint obs_pos_[10];

        DPoint ball_pos_;
        DPoint ball_vel_;
    public:
        bool   isinposition_;

    };
}
#endif // PLAN_H
