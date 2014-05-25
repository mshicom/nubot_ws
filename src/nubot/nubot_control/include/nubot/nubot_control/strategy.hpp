#ifndef STRATEGY_H
#define STRATEGY_H
#include <cmath>
#include"nubot/core/core.hpp"
#include "nubot/nubot_control/plan.hpp"
//#include "nubot/nubot_control/define.hpp"


//using namespace std;
namespace nubot{
    class Strategy
    {
    public:
        Strategy();
    public:

        unsigned char m_matchmode_;
        unsigned char m_mathtype_;
        unsigned char currentstate_;

               Plan   m_plan_;             //plan



               //DPoint obs_vel_;

               bool   dribble_state_;      //dribble state
               bool   quick_shoot_state_;       // quick shoot
               int    quick_shoot_count_; // count for quick shoot
               bool   dynamic_shoot_state_;
               int    dynamic_shoot_count_;

               float  leftdelta_for_shoot_;
               float  rightdelta_for_shoot_;

               DPoint opp_goal_pos_;
               bool   NeedEvaluat;         //check the flag to determine if the shoot program need
                                           //evaluation
               int    stucktime_;



               int    searchball_active_;

               bool   running;

               bool   game_begin_;
               DPoint ballpos_record_;
               float  distance2ball_record_;


               WolrdModeliInfo* worldmodelinfo_;

           public:

               unsigned char checkLocationRegion(DPoint location);  //

               bool IsLocationInOurField(DPoint location);
               bool IsLocationInOppField(DPoint location);
               bool IsLocationInOurPenalty(DPoint location);
               bool IsLocationInOppPenalty(DPoint location);
               bool IsLocationInOppGoalArea(DPoint location);
               bool IsLocationInField(DPoint location);

               bool checkShootDirection(float direction,float swidth,float lwidth,float len);
               bool evaluateKick(DPoint &target,float &leftdelta,float &rightdelta);
               bool checkTeamateDribble();

               bool checkRobotDrible();
               bool checkPermmitedMovement();

               void runGame();
               void stopGame();

               void activeDecisionMaking();    // active decision progress
               unsigned char currentstateSelection();
               void currentactionselection(unsigned char  state);
               void activeCatchBall();
               void avoidObs();
               void triggerShoot(DPoint target);
               void quickShoot();
               void turn4Shoot();
               void dynamicQuickShoot();
               void clearActiveState();
               void findBall();
               void kickball4Coop(DPoint target);


               DPoint defencepointCalculate();

               void positionforPlaceKicks();   // placekicks
               void coopforPlaceKicks();
               void stuckProcess(int stucktime);

    };
}
#endif // STRATEGY_H
