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

        //subtargets
        int
        Min_num(int n,double *q);
        double
        Min(int n,double *q);
        int
        Max_num(int n,double *q);
        double
        Max(int n,double *q);
        void
        subtarget(double *obs,double *pos_robot,double *pos_target,double *pos_subtarget);
        void
        subtargets(DPoint target);


        
    public:
        Behaviour m_behaviour_;
        WolrdModeliInfo* worldmodelinfo_;
        DPoint   subtargets_pos_;


    public:
        bool   isinposition_;

    };
}
#endif // PLAN_H
