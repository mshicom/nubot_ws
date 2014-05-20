#ifndef BALL_HANDLE_H
#define BALL_HANDLE_H

#include <ros/ros.h>
#include <boost/circular_buffer.hpp>

// 动态参数服务器
#include <dynamic_reconfigure/server.h>
#include <nubot_standalone/controllerConfig.h>

#include <realtime_tools/realtime_publisher.h>
#include "nubot_standalone/DebugInfo.h"
#include "nubot_standalone/VelCmd.h"
#include "nubot_HWController/cmac.h"


using namespace std;

class Nubot_HWController
{
public:
    Nubot_HWController(int argc,char** argv,const std::string &name);
    ~Nubot_HWController();
    bool DribbleParamRead(void);
    bool DribbleParamCalibrate(void);
    bool DribbleGetState(void);
    void DribbleController();
    void Timer1_Process(const ros::TimerEvent &event);
    //void BallLockEnable(bool on);
    void Read_VelCmd(const nubot_standalone::VelCmd::ConstPtr& cmd);
    void ParamChanged(nubot_HWController::controllerConfig &config, uint32_t level);
    void BaseController(/*const ros::TimerEvent &event*/);
private:
    ros::NodeHandle n;
    ros::Timer timer1;

    // 动态参赛服务器
    dynamic_reconfigure::Server<nubot_HWController::controllerConfig> server;
    dynamic_reconfigure::Server<nubot_HWController::controllerConfig>::CallbackType f;

    realtime_tools::RealtimePublisher<nubot_standalone::DebugInfo> *DebugInfo_pub;
    realtime_tools::RealtimePublisher<nubot_standalone::VelCmd> *OdeInfo_pub;

    ros::Subscriber Velcmd_sub_;

public:

    CMAC  cmac;
    //double cmac_ip[3],cmac_op[2],cmac_delta[2];

    int32 motor_speed[2];
    int32 &motor_left,&motor_right;
    int32 Motor_Readings[4];

    double Vx,Vy,w,Vx_diff;
    double Real_Vx,Real_Vy,Real_w;
    double FBRatio,FFRatio;
    double P,I,D;
    double Kx_f,Kx_b;

    // PID控制器的反馈误差历史
    boost::circular_buffer<double> e_l,e_r;
    double LeverPos_ErrL,LeverPos_ErrR;
    // 速度指令历史
    boost::circular_buffer<double> Vxs;


    double LeverPos_Left,LeverPos_Right,LeverPos_Diff;
    double LeverPos_SetPoint;
    double BallSensor_LBias,BallSensor_RBias,BallSensor_LStroke,BallSensor_RStroke;
    bool IsBallSensorInited;
    bool BallSensor_IsHolding,IsBallLost;
};


#endif
