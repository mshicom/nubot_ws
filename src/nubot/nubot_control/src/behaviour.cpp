#include "nubot/nubot_control/behaviour.hpp"
using namespace nubot;

Behaviour::Behaviour()
{
 app_vx_ = 0;
 app_vy_ = 0;
 app_w_  = 0;
}

void
Behaviour::revDecoupleFromVel(float vx,float vy,float &positivemax_rev,float &negativemax_rev)
{
    float v[4];
    float rate =  RATE;

    v[0] = (-0.707*vy + 0.707*vx)*rate;
    v[1] =  0.707*(vx + vy)*rate;
    v[2] =  0.707*(vy - vx)*rate;
    v[3] = (-0.707*vx - 0.707*vy)*rate;



    float maxvel =  -LIMITEDRPM;
    float minvel =  LIMITEDRPM;

    for (int i=0;i<2;i++)
    {
        if (v[i]>maxvel)
        {
            maxvel = v[i];
        }
        if (v[i]<minvel)
        {
            minvel = v[i];
        }
    }

    positivemax_rev = 0;
    negativemax_rev = 0;



    positivemax_rev = (LIMITEDRPM*FRICONRATIO-maxvel)/(rate*CHASSISRADIUS);
    negativemax_rev = (-LIMITEDRPM*FRICONRATIO-minvel)/(rate*CHASSISRADIUS);
}
float
Behaviour::basicPDControl(float pgain,float dgain, float err,float err1, float maxval)
{

    float _e1 = err1;
    float kp =  pgain;
    float kd=  dgain;
    float _e=err;
    float retval = 0;
    //float Tk = 33;
    //float p0,p1;
    //p0 = kp;
    //p1 = kp*kd/Tk;
    retval  = kp *_e +kd*(_e -_e1) ;

    if(fabs(retval) > maxval)
    {
        if(retval>0) retval= maxval;
        else    retval=-maxval;
    }
    return retval;
}
float
    Behaviour::basicPIDcontrol(float pgain,
                   float igain,
                   float dgain,
                   float currval,
                   float targetval,
                   float imaxlimiter,
                   float iminlimiter,
                   float& dstate,
                   float& istate )
{
    float retval=0;
    //common PID control implementation with i-part-limitation
    float pterm=0, dterm, iterm;
    float error = targetval-currval;

    pterm = pgain * error;   // calculate the proportional term

    // calculate the integral state with appropriate limiting
    istate += error;
    if (istate > imaxlimiter) {
        //     cerr << "Caution: I_MAX value trespassed" << endl;
        istate = imaxlimiter;
    }
    else if (istate < iminlimiter) {
        istate = iminlimiter;
        // cerr << "Caution: I_MIN value trespassed" << endl;
    }//ENDELSE

    iterm = igain * istate;  // calculate the integral term

    dterm = dgain * (dstate - targetval);
    dstate = targetval;

    retval= currval+ pterm + dterm + iterm;
    return retval;
}
// omnidirectional mobile robot control model

// activedriible control device control model

// dribble control   //basic control methods



// tranlation control
void
Behaviour::move2Position(float pval, float dval, DPoint target, float maxvel)  // move to the target point by using PD control
{

    float _pos_e = worldmodelinfo_.robot_pos_.distance(target);
    DPoint relposoftarget =  target  - worldmodelinfo_.robot_pos_;
    // DPoint2f(target.x_ - m_worldmodel_->worldmodelinfo_.robot_pos_.x_, target.y_- m_worldmodel_->worldmodelinfo_.robot_pos_.y_);

    float tar_theta = relposoftarget.angle().radian_;  //

    static float _pos_e1 = 0;

    float speed  = 0;


     speed = basicPDControl(pval,dval,_pos_e,_pos_e1,maxvel);


    //m_worldmodel_->worldmodelinfo_.robot_ori_.radian_ = tar_theta;

    app_vx_ =  speed*cos(tar_theta - worldmodelinfo_.robot_ori_.radian_);  //
    app_vy_ =  speed*sin(tar_theta - worldmodelinfo_.robot_ori_.radian_);

    double v=sqrt(app_vx_*app_vx_+app_vy_*app_vy_);

    if(v>maxvel)
    {
        app_vx_=app_vx_*maxvel/v;
        app_vy_=app_vy_*maxvel/v;
    }
    _pos_e1  = _pos_e;

}
void
Behaviour::move2target(float pval, float dval,DPoint target, DPoint realtarvel, float maxvel)
{
    float _pos_e = worldmodelinfo_.robot_pos_.distance(target);
    DPoint relposoftarget =  target  - worldmodelinfo_.robot_pos_;
    // DPoint2f(target.x_ - m_worldmodel_->worldmodelinfo_.robot_pos_.x_, target.y_- m_worldmodel_->worldmodelinfo_.robot_pos_.y_);

    float tar_theta = relposoftarget.angle().radian_;  //

    static float _pos_e1 = 0;

    float speed  = 0;

    float  pos_e_threshold = 150;   // ?

    if(_pos_e>pos_e_threshold)
          speed = basicPDControl(pval,dval,_pos_e,_pos_e1,maxvel);
    else
         speed = basicPDControl(pval*0.75,dval*0.7,_pos_e,_pos_e1,maxvel);

    //m_worldmodel_->worldmodelinfo_.robot_ori_.radian_ = tar_theta;

    DPoint compensation_vel = realtarvel;


    app_vx_ =  speed*cos(tar_theta - worldmodelinfo_.robot_ori_.radian_) + compensation_vel.x_;  //
    app_vy_ =  speed*sin(tar_theta - worldmodelinfo_.robot_ori_.radian_) + compensation_vel.y_;

    double v=sqrt(app_vx_*app_vx_+app_vy_*app_vy_);
    if(v>maxvel)
    {
        app_vx_=app_vx_*maxvel/v;
        app_vy_=app_vy_*maxvel/v;
    }
    _pos_e1  = _pos_e;
}
void
Behaviour::bangbangSolution()
{

}
void
Behaviour::bangbangmove2Position(DPoint target)
{

}
void
Behaviour::traceTarget()
{

}

// rotation control
void
Behaviour::rotate2AbsOrienation(float pval, float dval, float orientation,float maxw)
{

    float theta_e = orientation-  worldmodelinfo_.robot_ori_.radian_;
    static float theta_e1 =  0;
    //static double theta_e2 =  0;
    if(theta_e > SINGLEPI_CONSTANT) theta_e = theta_e-2*SINGLEPI_CONSTANT;
    if(theta_e<=-SINGLEPI_CONSTANT) theta_e = theta_e+2*SINGLEPI_CONSTANT;

    //w = BasicPIDControl_Delta(w,kp,ki,kd,theta_e,theta_e1,theta_e2,0.05*pi,max_omega);
    app_w_ = basicPDControl(pval,dval,theta_e,theta_e1,maxw);

    float  positivemax_rev;
    float  negativemax_rev;

    revDecoupleFromVel(app_vx_,app_vy_,positivemax_rev,negativemax_rev);

    if (app_w_  > positivemax_rev)
        app_w_  = positivemax_rev;
    else if ( app_w_  < negativemax_rev)
        app_w_  = negativemax_rev;
  //printf("negativemax_rev :  %f, negativemax_rev:   %f ",positivemax_rev,negativemax_rev);
    theta_e1 = theta_e ;

}// rotate to the target orientation by using PD control
void
Behaviour::rotate2RelOrienation(float pval, float dval, float rel_orientation,float maxw)
{

}
void
Behaviour::rotatetowardsSetPoint(DPoint point)
{

}
void
Behaviour::rotatetowardsRelPoint(DPoint rel_point)
{

}

// accelaration process
void
Behaviour::accelerateProcess(DPoint &vr ,double acc,double threshhold)
{

}

void Behaviour::clearBehaviorState()
{

    app_vx_ = 0;
    app_vy_ = 0;
    app_w_  = 0;

}
