#include "nubot/nubot_control/plan.hpp"
#include "nubot/core/core.hpp"
using namespace  nubot;
nubot::Plan::Plan()
{

    robot_pos_  =  m_behaviour_.robot_pos_;
    robot_ori_  =  m_behaviour_.robot_ori_;
    robot_vel_  =  m_behaviour_.robot_vel_;
    ball_pos_   =  m_behaviour_.ball_pos_;
    ball_vel_   =  m_behaviour_.ball_vel_;

    isinposition_ = false;
}

void
nubot::Plan::traceBall()
{

}

void
nubot::Plan::interceptBall()  // intercept
{
   DPoint robotglobalvel = robot_vel_;  //global velocity ?
   DPoint ballglobalvel  = ball_vel_;

}

void
nubot::Plan::catchBallbyPN()
{

}

void
nubot::Plan::catchBall()
{

    float ballspeed = sqrt(ball_vel_.x_*ball_vel_.x_+ ball_vel_.y_*ball_vel_.y_);

    if(ballspeed < LIMITEDBALLSPEED)
    {
        catchMotionlessBall();
    }
    else
        catchMovingBall();

}

void
nubot::Plan::catchMovingBall()
{

    float  thetaofr2b =  robot_pos_.angle(ball_pos_).radian_;
    float  thetaofb2r =  ball_pos_.angle(robot_pos_).radian_;

    //DPoint rbv = robot_pos_ - ball_pos_;



    float  delta_theta1  = thetaofb2r - robot_ori_.radian_;

    angularnorm(delta_theta1);

    float delta_theta2 =  thetaofr2b - ball_vel_.angle().radian_;

    angularnorm(delta_theta2);

    float disofr2b = robot_pos_.distance(ball_pos_);

    float in_vwkp  = 1.5, in_vwkd = 1.2;
    float in_vkp  = 1.5, in_vkd = 1.2;
    float in_wkp  = 1.5, in_wkd = 1.2;

    float out_vwkp  = 1.5, out_vwkd = 1.2;

    float out1_vkp  = 1.5, out1_vkd = 1.2;
    float out1_wkp  = 1.5, out1_wkd = 1.2;

    float out2_vkp  = 1.5, out2_vkd = 1.2;
    float out2_wkp  = 1.5, out2_wkd = 1.2;


    //static int state =  0;
   // static double last_delta_theta = 0;


    DPoint catballvel = DPoint(0.0,0.0);
    //*/
       float DISTANCE = 150;
    //*/
    DPoint tangentvel;



   float turnangle  =  ball_vel_.angle().radian_ + SINGLEPI_CONSTANT;

   angularnorm(turnangle);

   DPoint compensation_vel = DPoint(50*cos(thetaofr2b),50*sin(thetaofr2b));

  if(fabs(delta_theta2 > SINGLEPI_CONSTANT/2))
  {
       float err = signbit(delta_theta2)*(ball_vel_.angle().radian_ - thetaofb2r);

       angularnorm(err);

       static float err1 = 0;

       float _w =  m_behaviour_.basicPDControl(in_vwkp,in_vwkd,err,err1,MAXW);

       catballvel = DPoint(65+ball_vel_.x_,ball_vel_.y_);

       float thetap = 0;

       float temtheta = thetaofr2b - ball_vel_.angle().radian_;

       angularnorm(temtheta);


       if(temtheta < 0 && temtheta >  -SINGLEPI_CONSTANT/2)
        thetap = thetaofr2b - SINGLEPI_CONSTANT/2;
       else if(temtheta > 0 && temtheta < SINGLEPI_CONSTANT/2)
        thetap = thetaofr2b + SINGLEPI_CONSTANT/2;
       else if(temtheta > SINGLEPI_CONSTANT/2 && temtheta < SINGLEPI_CONSTANT)
        thetap = thetaofr2b - SINGLEPI_CONSTANT/2;
       else
        thetap = thetaofr2b + SINGLEPI_CONSTANT/2;

        angularnorm(thetap);


       tangentvel = DPoint(_w*disofr2b*cos(thetap),_w*disofr2b*sin(thetap));

       catballvel = catballvel + tangentvel;

       DPoint realballvel = DPoint(catballvel.x_*cos(robot_ori_.radian_)+catballvel.y_*sin(robot_ori_.radian_),
                                   -catballvel.x_*sin(robot_ori_.radian_)+catballvel.y_*cos(robot_ori_.radian_));

       m_behaviour_.move2target(in_vkp,in_vkd,ball_pos_,realballvel,MAXVEL);

       m_behaviour_.rotate2AbsOrienation(in_wkp,in_wkd,thetaofr2b,MAXW);

       err1 = err;

  }
  else
  {
      if(fabs(delta_theta2) < SINGLEPI_CONSTANT/18 && disofr2b > DISTANCE)
       catballvel = DPoint(100+ball_vel_.x_,ball_vel_.y_);
      else
       catballvel = DPoint(100+ball_vel_.x_,ball_vel_.y_);

      float err =  signbit(delta_theta2)*delta_theta2;
      static float err1 = 0;
      float _w =  m_behaviour_.basicPDControl(out_vwkp,out_vwkd,err,err1,MAXW);
      float thetap =0;
      float temtheta =  thetaofr2b -  ball_vel_.angle().radian_;

      angularnorm(temtheta);

      if(temtheta <  0 && temtheta >  -SINGLEPI_CONSTANT/2)
      thetap = thetaofr2b - SINGLEPI_CONSTANT/2;
      else if(temtheta  >  0 && temtheta <  SINGLEPI_CONSTANT/2)
      thetap = thetaofr2b + SINGLEPI_CONSTANT/2;
      else if(temtheta  > SINGLEPI_CONSTANT/2 && temtheta <  SINGLEPI_CONSTANT)
      thetap = thetaofr2b - SINGLEPI_CONSTANT/2;
      else
      thetap = thetaofr2b + SINGLEPI_CONSTANT/2;

      angularnorm(thetap);

      tangentvel = DPoint(_w*disofr2b*cos(thetap),_w*disofr2b*sin(thetap));
      catballvel = catballvel + tangentvel;

      DPoint realballvel = DPoint(catballvel.x_*cos(robot_ori_.radian_)+catballvel.y_*sin(robot_ori_.radian_),
                                  -catballvel.x_*sin(robot_ori_.radian_)+catballvel.y_*cos(robot_ori_.radian_));

      if(disofr2b >  DISTANCE)
      {
          if(fabs(robot_ori_.radian_ - thetaofr2b) < SINGLEPI_CONSTANT/18)
          {
             m_behaviour_.move2target(out1_vkp,out1_vkd,ball_pos_,realballvel,MAXVEL);
             m_behaviour_.rotate2AbsOrienation(out1_wkp,out1_wkd,thetaofr2b,MAXW);

          }
          else
          {
              m_behaviour_.move2target(out1_vkp,out1_vkd,ball_pos_,realballvel,MAXVEL-150);
              m_behaviour_.rotate2AbsOrienation(out1_wkp,out1_wkd,thetaofr2b,MAXW);
          }
      }
      else
      {
          m_behaviour_.move2target(out2_vkp,out2_vkd,ball_pos_,realballvel,MAXVEL);
          m_behaviour_.rotate2AbsOrienation(out2_wkp,out2_wkd,thetaofr2b,MAXW);
      }
      err1  = err;
   }
 }


void
nubot::Plan::catchMotionlessBall()
{
   float  thetaofr2b =  robot_pos_.angle(ball_pos_).radian_;
   //DPoint rbv = robot_pos_ - ball_pos_;
   float  delta_theta = thetaofr2b - robot_ori_.radian_;

   angularnorm(delta_theta);

   float disofr2b = robot_pos_.distance(ball_pos_);

   static int state =  0;
   static double last_delta_theta = 0;

   DPoint catballvel = DPoint(0.0,0.0);
   DPoint realballvel = DPoint(ball_vel_.x_*cos(robot_ori_.radian_)+ball_vel_.y_*sin(robot_ori_.radian_),
                               -ball_vel_.x_*sin(robot_ori_.radian_)+ball_vel_.y_*cos(robot_ori_.radian_));
   float vtr0kp = 1.5, vtr1kp = 1.2, vtr2kp = 1.4;
   float vtr0kd = 1.6, vtr1kd = 1.2, vtr2kd = 1.5 ;
   float wtr0kp = 1.2, wtr1kp = 1.2, wtr2kp = 1.2;
   float wtr0kd = 1.1, wtr1kd = 1.1, wtr2kd = 1.1;


//*/
   float DISTANCE = 150;
//*/

  if(delta_theta > SINGLEPI_CONSTANT/3)
      state = 1;
  else if (disofr2b < DISTANCE)
      state  = 0;

   last_delta_theta = delta_theta;
   float compensation_speed = 50;

   DPoint compensation_vel = DPoint(compensation_speed*cos(thetaofr2b),compensation_speed*sin(thetaofr2b));


   if(state ==  0)
   {
       catballvel = realballvel;   // translate into the body coordinnate

       m_behaviour_.move2target(vtr0kp,vtr0kd,ball_pos_,catballvel,MAXVEL);

       m_behaviour_.rotate2AbsOrienation(wtr0kp,wtr0kd,thetaofr2b,MAXW);

   }
   else if(state == 1)
   {
       catballvel = realballvel + compensation_vel;   // translate into the body coordinnate

       m_behaviour_.move2target(vtr1kp,vtr1kd,ball_pos_,catballvel,MAXVEL);

       m_behaviour_.rotate2AbsOrienation(wtr1kp,wtr1kd,thetaofr2b,MAXW);
   }
   else
   {

       catballvel = realballvel + compensation_vel;   // translate into the body coordinnate

       m_behaviour_.move2target(vtr1kp,vtr1kd,ball_pos_,catballvel,MAXVEL);

       m_behaviour_.rotate2AbsOrienation(wtr1kp,wtr1kd,thetaofr2b,MAXW);

   }

   /*no dribble control*/

}


void
nubot::Plan::positionAvoidObs(DPoint target, float theta, float stopdis, float stoptheta)
{


        float distar2self  =  robot_pos_.distance(target);

        if(distar2self < stopdis && fabs(theta) < stoptheta)
            isinposition_ = true;

        if(!isinposition_)
        {
            avoidRelOble(target,theta,OBLE_RADIUS,POSITION_LIMIT_VEL,1);//
        }
        else
        {


            m_behaviour_.clearBehaviorState();

            if(distar2self>stopdis+LOCATIONERROR|| fabs(theta)>3*stoptheta)
                isinposition_=0;
        }

}

nubot::DPoint
nubot::Plan::avoidRelOble(DPoint target, double theta, double ro, double vel, bool includeball)
{
    /*DPoint Obstacle[Max_ObsVision+2][Max_ObsVision+2];
    int    RobotNum[Max_ObsVision+2];//={0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    memset(RobotNum,0,(Max_ObsVision+2)*sizeof(int));
    DPoint Robot[Max_ObsVision+2];
    DPoint Oble[Max_ObsVision+2];
    int    AllObleNum=0;
    double Dis=robot_pos_.distance(target);

    int numOfObsVision=m_WorldModel->Num_ObsVision;
    for(int i=0;i<numOfObsVision;i++)
        Oble[i]=m_WorldModel->OtherRobot[i].RelPos;
    for(int i=numOfObsVision;i<numOfObsVision+2;i++)
    {
        Oble[i]=m_WorldModel->ObleOppGoalCorner[i-numOfObsVision].RelPos;

    }
    for(int i=numOfObsVision+2;i<Max_ObsVision+2;i++)
        Oble[i]=DPoint(0,0);

    CleanUpObl(Robot,Oble);

    int  k=0;
    DetectObles(target,Robot,Obstacle,RobotNum,AllObleNum,RO,IncludeBall);
    DPoint foundtarget=FindBestDrection2(target,Obstacle,RobotNum,RO,AllObleNum,k);
    double ttheta=atan2(foundtarget.y,foundtarget.x);
    foundtarget=DPoint(Dis*cos(ttheta),Dis*sin(ttheta));
    RelTarget=foundtarget;

    m_Behaviour.Move2RelPos(foundtarget,theta,Vel);	*/
    DPoint foundtarget;
    return foundtarget;
}
