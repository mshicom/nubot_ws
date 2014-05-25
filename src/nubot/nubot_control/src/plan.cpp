#include "nubot/nubot_control/plan.hpp"
#include "nubot/core/core.hpp"
using namespace  nubot;
nubot::Plan::Plan()
{

    isinposition_ = false;

    worldmodelinfo_ = &(m_behaviour_.worldmodelinfo_);
}

void
nubot::Plan::traceBall()
{

}
/*
 * intercept ball based on PN guidance law!
 *
 */
void
nubot::Plan::interceptBall()  // intercept
{
   DPoint robotglobalpos = worldmodelinfo_->robot_pos_;
   DPoint robotglobalvel = worldmodelinfo_->robot_vel_;  //

   DPoint ballglobalpos  = worldmodelinfo_->ball_pos_;
   DPoint ballglobalvel  = worldmodelinfo_->ball_vel_;

   double distance = robotglobalpos.distance(ballglobalpos);
   double speed = sqrt(robotglobalvel.x_*robotglobalvel.x_+robotglobalvel.y_*robotglobalvel.y_);
   double a1 = (ballglobalvel.x_ - robotglobalvel.x_)*(robotglobalpos.y_-ballglobalpos.y_);
   double a2 = (ballglobalpos.x_-robotglobalpos.x_)*(ballglobalvel.y_-robotglobalvel.y_);
   double deltaq = (a1-a2)/(distance*distance);
   double N = 2.5;
   double acc = N*deltaq*speed;

   double theta1 = robotglobalvel.angle().radian_;
   double theta2 = theta1 + SINGLEPI_CONSTANT/2;
          theta2 = angularnorm(theta2);
   DPoint globaldeltavel = DPoint(acc*cos(theta2),acc*sin(theta2));  // global
   DPoint reldeltavel  = vglobal2rel(globaldeltavel,worldmodelinfo_->robot_ori_.radian_);
   m_behaviour_.app_vx_ += reldeltavel.x_;
   m_behaviour_.app_vy_ += reldeltavel.y_;
}

void
nubot::Plan::catchBallbyPN()
{

}

void
nubot::Plan::catchBall()
{

    float ballspeed = sqrt(worldmodelinfo_->ball_vel_.x_*worldmodelinfo_->ball_vel_.x_+ worldmodelinfo_->ball_vel_.y_*worldmodelinfo_->ball_vel_.y_);

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

    float  thetaofr2b =  worldmodelinfo_->robot_pos_.angle(worldmodelinfo_->ball_pos_).radian_;
    float  thetaofb2r =  worldmodelinfo_->ball_pos_.angle(worldmodelinfo_->robot_pos_).radian_;

    //DPoint rbv = robot_pos_ - ball_pos_;



    float  delta_theta1  = thetaofb2r - worldmodelinfo_->robot_ori_.radian_;

    angularnorm(delta_theta1);

    float delta_theta2 =  thetaofr2b - worldmodelinfo_->ball_vel_.angle().radian_;

    angularnorm(delta_theta2);

    float disofr2b = worldmodelinfo_->robot_pos_.distance(worldmodelinfo_->ball_pos_);

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



   float turnangle  =  worldmodelinfo_->ball_vel_.angle().radian_ + SINGLEPI_CONSTANT;

   angularnorm(turnangle);

   DPoint compensation_vel = DPoint(50*cos(thetaofr2b),50*sin(thetaofr2b));

  if(fabs(delta_theta2 > SINGLEPI_CONSTANT/2))
  {
       float err = signbit(delta_theta2)*(worldmodelinfo_->ball_vel_.angle().radian_ - thetaofb2r);

       angularnorm(err);

       static float err1 = 0;

       float _w =  m_behaviour_.basicPDControl(in_vwkp,in_vwkd,err,err1,MAXW);

       catballvel = DPoint(65+worldmodelinfo_->ball_vel_.x_,worldmodelinfo_->ball_vel_.y_);

       float thetap = 0;

       float temtheta = thetaofr2b - worldmodelinfo_->ball_vel_.angle().radian_;

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

       DPoint realballvel = DPoint(catballvel.x_*cos(worldmodelinfo_->robot_ori_.radian_)+catballvel.y_*sin(worldmodelinfo_->robot_ori_.radian_),
                                   -catballvel.x_*sin(worldmodelinfo_->robot_ori_.radian_)+catballvel.y_*cos(worldmodelinfo_->robot_ori_.radian_));

       m_behaviour_.move2target(in_vkp,in_vkd,worldmodelinfo_->ball_pos_,realballvel,MAXVEL);

       m_behaviour_.rotate2AbsOrienation(in_wkp,in_wkd,thetaofr2b,MAXW);

       err1 = err;

  }
  else
  {
      if(fabs(delta_theta2) < SINGLEPI_CONSTANT/18 && disofr2b > DISTANCE)
       catballvel = DPoint(100+worldmodelinfo_->ball_vel_.x_,worldmodelinfo_->ball_vel_.y_);
      else
       catballvel = DPoint(100+worldmodelinfo_->ball_vel_.x_,worldmodelinfo_->ball_vel_.y_);

      float err =  signbit(delta_theta2)*delta_theta2;
      static float err1 = 0;
      float _w =  m_behaviour_.basicPDControl(out_vwkp,out_vwkd,err,err1,MAXW);
      float thetap =0;
      float temtheta =  thetaofr2b -  worldmodelinfo_->ball_vel_.angle().radian_;

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

      DPoint realballvel = DPoint(catballvel.x_*cos(worldmodelinfo_->robot_ori_.radian_)+catballvel.y_*sin(worldmodelinfo_->robot_ori_.radian_),
                                  -catballvel.x_*sin(worldmodelinfo_->robot_ori_.radian_)+catballvel.y_*cos(worldmodelinfo_->robot_ori_.radian_));

      if(disofr2b >  DISTANCE)
      {
          if(fabs(worldmodelinfo_->robot_ori_.radian_ - thetaofr2b) < SINGLEPI_CONSTANT/18)
          {
             m_behaviour_.move2target(out1_vkp,out1_vkd,worldmodelinfo_->ball_pos_,realballvel,MAXVEL);
             m_behaviour_.rotate2AbsOrienation(out1_wkp,out1_wkd,thetaofr2b,MAXW);

          }
          else
          {
              m_behaviour_.move2target(out1_vkp,out1_vkd,worldmodelinfo_->ball_pos_,realballvel,MAXVEL-150);
              m_behaviour_.rotate2AbsOrienation(out1_wkp,out1_wkd,thetaofr2b,MAXW);
          }
      }
      else
      {
          m_behaviour_.move2target(out2_vkp,out2_vkd,worldmodelinfo_->ball_pos_,realballvel,MAXVEL);
          m_behaviour_.rotate2AbsOrienation(out2_wkp,out2_wkd,thetaofr2b,MAXW);
      }
      err1  = err;
   }
 }


void
nubot::Plan::catchMotionlessBall()
{
   float  thetaofr2b =  worldmodelinfo_->robot_pos_.angle(worldmodelinfo_->ball_pos_).radian_;
   //DPoint rbv = robot_pos_ - ball_pos_;
   float  delta_theta = thetaofr2b - worldmodelinfo_->robot_ori_.radian_;

   angularnorm(delta_theta);

   float disofr2b = worldmodelinfo_->robot_pos_.distance(worldmodelinfo_->ball_pos_);

   static int state =  0;
   static double last_delta_theta = 0;

   DPoint catballvel = DPoint(0.0,0.0);
   DPoint realballvel = DPoint(worldmodelinfo_->ball_vel_.x_*cos(worldmodelinfo_->robot_ori_.radian_)+worldmodelinfo_->ball_vel_.y_*sin(worldmodelinfo_->robot_ori_.radian_),
                               -worldmodelinfo_->ball_vel_.x_*sin(worldmodelinfo_->robot_ori_.radian_)+worldmodelinfo_->ball_vel_.y_*cos(worldmodelinfo_->robot_ori_.radian_));
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

       m_behaviour_.move2target(vtr0kp,vtr0kd,worldmodelinfo_->ball_pos_,catballvel,MAXVEL);

       m_behaviour_.rotate2AbsOrienation(wtr0kp,wtr0kd,thetaofr2b,MAXW);

   }
   else if(state == 1)
   {
       catballvel = realballvel + compensation_vel;   // translate into the body coordinnate

       m_behaviour_.move2target(vtr1kp,vtr1kd,worldmodelinfo_->ball_pos_,catballvel,MAXVEL);

       m_behaviour_.rotate2AbsOrienation(wtr1kp,wtr1kd,thetaofr2b,MAXW);
   }
   else
   {

       catballvel = realballvel + compensation_vel;   // translate into the body coordinnate

       m_behaviour_.move2target(vtr1kp,vtr1kd,worldmodelinfo_->ball_pos_,catballvel,MAXVEL);

       m_behaviour_.rotate2AbsOrienation(wtr1kp,wtr1kd,thetaofr2b,MAXW);

   }

   /*no dribble control*/

}


void
nubot::Plan::positionAvoidObs(DPoint target, float theta, float stopdis, float stoptheta)
{


        float distar2self  =  worldmodelinfo_->robot_pos_.distance(target);

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

double
nubot::Plan::Max(int n, double *q)
{
    int i=0;
    double max_out=q[0];
    for(i=1;i<n;i++)
    {
        if(q[i]>max_out) max_out=q[i];
    }
    return max_out;
}

int
nubot::Plan::Max_num(int n,double *q)
{
    int i=0,max_out_num=0;
    for(i=1;i<n;i++)
    {
        if(q[i]>q[max_out_num]) max_out_num=i;
    }
    return max_out_num;
}

double
nubot::Plan::Min(int n,double *q)
{
    int i=0;
    double min_out=q[0];
    for(i=1;i<n;i++)
    {
        if(q[i]<min_out) min_out=q[i];
    }
    return min_out;
}

int
nubot::Plan::Min_num(int n,double *q)
{
    int i=0,min_out_num=0;
    for(i=1;i<n;i++)
    {
        if(q[i]<q[min_out_num]) min_out_num=i;
    }
    return min_out_num;
}

void
nubot::Plan::subtarget(double *obs,double *pos_robot,double *pos_target,double *pos_subtarget)
{
    double radius_robot=40,radius_obs=40;

    double a[9],b[9];
    double x=0;
    double y=0;
    int i=0,j=0,k=0;
    int B[9]={0};
    int First_num=0;
    double minB=0;

    int G[9]={-1,-1,-1,-1,-1,-1,-1,-1,-1};
    int G_num=0;
    int G_obs[9]={1,1,1,1,1,1,1,1,1};

    double b_positive[10]={0};
    double b_negative[10]={0};
    int left=0,right=0,sign_side=0,bp_num=0,bn_num=0;

    double atemp=0;
    double alpha[9]={0};
    double alpha_i=0;
    int alpha_k=0;

    bool canpass=0;

    x=pos_target[0]-pos_robot[0];
    y=pos_target[1]-pos_robot[1];

    //obtain(ai,bi)--position in the direction of target
    for(i=0;i<9;i++)
    {
        double x1=obs[2*i]-pos_robot[0];
        double y1=obs[2*i+1]-pos_robot[1];
        int temp=0;
        if((x*y1-x1*y)==0)
            temp=0;
        else if((x*y1-x1*y)>0)
            temp=1;
        else temp=-1;

        a[i]=(x1*x+y1*y)/sqrt(x*x+y*y);
        b[i]=temp*fabs(x*y1-x1*y)/sqrt(x*x+y*y);
    }
    //obtain B that may hit
    for(i=0;i<9;i++)
    {
        if((a[i]>0)&&(a[i]<sqrt(x*x+y*y))&&(fabs(b[i])<(radius_robot+radius_obs)))
        {
            B[j]=i;
            j++;
        }
    }

    if(j!=0)
    {
        //determain first obs
        First_num=B[0];
        minB=a[B[0]];
        for(i=1;i<j;i++)
        {
            if(minB<a[B[i]]) minB=minB;
            else
            {
                minB=a[B[i]];
                First_num=B[i];
            }
        }

        //Grouping--the obs that must be avoided
        G[0]=First_num;
        G_num=0;
        G_obs[First_num]=0;

        for(i=0;i<9;i++)
        {
            if(G_obs[i]==1)
            {
                for(k=0;k<=G_num;k++)
                {
                    if((sqrt(pow((obs[2*i]-obs[2*G[k]]),2)+pow((obs[2*i+1]-obs[2*G[k]+1]),2)))<(2*radius_robot+2*radius_obs))
                    {
                        G_num++;
                        G[G_num]=i;
                        G_obs[i]=0;
                        i=-1;
                        break;
                    }
                }
            }
        }
        //Location of subtarget
        for(i=0;i<=G_num;i++)
        {
            if(b[G[i]]>0)
            {
                bp_num++;
                b_positive[bp_num]=b[G[i]];
            }
            else if(b[G[i]]<0)
            {
                bn_num++;
                b_negative[bn_num]=fabs(b[G[i]]);
            }
        }
        if(Max(10,b_positive)<=Max(10,b_negative))
        {
            left=1;
            sign_side=1;
        }
        else
        {
            right=1;
            sign_side=-1;
        }

        for(i=0;i<=G_num;i++)
        {
            atemp=sqrt((obs[2*G[i]]-pos_robot[0])*(obs[2*G[i]]-pos_robot[0])+(obs[2*G[i]+1]-pos_robot[1])*(obs[2*G[i]+1]-pos_robot[1]));
            if(atemp<(radius_robot+radius_obs))
            {
                atemp=radius_robot+radius_obs+0.0001;
                canpass=1;
            }
            alpha[i]=atan2(b[G[i]],a[G[i]])+sign_side*asin((radius_robot+radius_obs)/atemp);
        }


        if(left==1)
        {
            alpha_i=Max(G_num+1,alpha);
            alpha_k=Max_num(G_num+1,alpha);
        }
        else
        {
            alpha_i=Min(G_num+1,alpha);
            alpha_k=Min_num(G_num+1,alpha);
        }
        pos_subtarget[0]=pos_robot[0]+(cos(alpha_i)*x-sin(alpha_i)*y)*sqrt((obs[2*G[alpha_k]]-pos_robot[0])*(obs[2*G[alpha_k]]-pos_robot[0])+(obs[2*G[alpha_k]+1]-pos_robot[1])*(obs[2*G[alpha_k]+1]-pos_robot[1]))/sqrtl(x*x+y*y);
        pos_subtarget[1]=pos_robot[1]+(sin(alpha_i)*x+cos(alpha_i)*y)*sqrt((obs[2*G[alpha_k]]-pos_robot[0])*(obs[2*G[alpha_k]]-pos_robot[0])+(obs[2*G[alpha_k]+1]-pos_robot[1])*(obs[2*G[alpha_k]+1]-pos_robot[1]))/sqrtl(x*x+y*y);
    }
    else
    {
        pos_subtarget[0]=pos_target[0];
        pos_subtarget[1]=pos_target[1];
    }
}

void
nubot::Plan::subtargets(nubot::DPoint target)
{
    double obs[18]={0};
    double pos_robot[2]={0,0};
    double pos_target[2]={0,0};
    double pos_subtarget[2]={0,0};

    for(int i=0;i<9;i++)
    {
        obs[2*i]   = worldmodelinfo_->obs_pos_[i].x_;
        obs[2*i+1] = worldmodelinfo_->obs_pos_[i].y_;
    }

    pos_robot[0]  = worldmodelinfo_->robot_pos_.x_;
    pos_robot[1]  = worldmodelinfo_->robot_pos_.y_;
    pos_target[0] = target.x_;
    pos_target[1] = target.y_;

    subtarget(obs,pos_robot,pos_target,pos_subtarget);

    subtargets_pos_.x_  = pos_subtarget[0];
    subtargets_pos_.y_  = pos_subtarget[1];
}
