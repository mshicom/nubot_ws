#include <stdio.h>
#include <fstream>
#include <string>

// 终端
#include <ncurses.h>

// 系统中断
#include <unistd.h>
#include <sys/types.h>

#define _USE_MATH_DEFINES
#include <math.h>
#include <vector>

#include <ros/ros.h>
#include <boost/circular_buffer.hpp>
#include <boost/foreach.hpp>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

// C与C++混合编程要在CPP文件中加extern “C”关键字，否则链接会出错
#include "soem/EtherCAT.h"


// 动态参数服务器
#include <dynamic_reconfigure/server.h>
#include <nubot_HWController/controllerConfig.h>

#include "nubot_HWController/nubot_HWController_node.h"
#include "nubot_HWController/cmac.h"

#include "nubot_common/VelCmd.h"
#include "nubot_HWController/DebugInfo.h"
#include "nubot_common/Shoot.h"
#include "nubot_common/BallHandle.h"



#define CAN_SLOT_POSITION 4     // CAN模块位置

#ifndef min
    #define min(x, y) (((x) > (y)) ? (y) : (x))
    #define max(x, y) (((x) > (y)) ? (x) : (y))
#endif

#define deg(a) (PI*(a)/180.0f)
#define Limmit(Vmin,v,Vmax)			(min(max((v),(Vmin)),(Vmax)))

int32 zero4[4]={0,0,0,0};
int32 zero2[2]={0,0};

/*********************DIY的计数器***************************
// Example:隔10次触发一次输出:
//       static trigger<10> t; if(t()){printf(...)}
// *******************************************************/
template <size_t N>
class trigger
{
public:
    trigger() :cnt(0){};

    size_t cnt;
    bool operator() ()
    {
        if (++cnt<N)
            return false;
        else
        {  cnt=0; return true; }
    }
};

#include <boost/date_time/posix_time/ptime.hpp>
using namespace boost::posix_time;
class ticker
{
public:
    ptime start,stop;
    string name;
public:
    ticker(string name)
        :start(microsec_clock::local_time()),name(name)
    {	};
    ~ticker()
    {
        stop=microsec_clock::local_time();
        time_duration diff = stop - start;
        std::cout<<name<<':'<<diff.total_milliseconds() << std::endl;
    }
};


Nubot_HWController::Nubot_HWController()
    :n(),motor_left(motor_speed[0]),motor_right(motor_speed[1]),
      Vx(0),Vy(0),w(0),P(9.5),I(2),D(2.4),Kx_f(1.2),Kx_b(0.8),
      FBRatio(1.0),FFRatio(1.0),
      cmac(1, 2, 20, 100, 0),
      BallHandleEnable(false),ShootEnable(false),
      strand_(io),timer1_(io, boost::posix_time::millisec(1)),timer2_(io, boost::posix_time::millisec(30)),timer3_(io)
{
#if USE_CURSE
    initscr();
#endif

    f = boost::bind(&Nubot_HWController::ParamChanged, this, _1, _2);
    server.setCallback(f);

#if 0
    n.param<double>("forward_ratio", FFRatio, 0);
    n.param<double>("backward_ratio", FBRatio,  1);
    n.param<double>("P", P,  35);
    n.param<double>("dist", BallHoldingDistance,  0.7);
#endif

    DribbleParamRead();

    double imin[3]={-500,-1,-1};
    double imax[3]={ 500, 1, 1};
    cmac.SetInputRange(imin,imax);
    cmac.LoadTable("wtable.dat");


    // 初始化环形buffer
    e_l.assign(3,3,0);
    e_r.assign(3,3,0);
    Vxs.assign(4,4,0);

    try
    {
        EtherCAT_init("eth0");
    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << std::endl;
    }
    //Base_Enable(false);

    DebugInfo_pub = new realtime_tools::RealtimePublisher<nubot_HWController::DebugInfo>(n, "/nubotdriver/debug", 4);
    OdeInfo_pub = new realtime_tools::RealtimePublisher<nubot_common::VelCmd>(n, "/nubotdriver/odoinfo", 4);

    timer1=n.createTimer(ros::Rate(100),&Nubot_HWController::Timer1_Process,this);

    Velcmd_sub_ = n.subscribe("/Motion_Velcmd",0,&Nubot_HWController::Read_VelCmd,this);

    ballhandle_service_ = n.advertiseService("BallHandle",&Nubot_HWController::BallHandleControlService,this);
    shoot_service_ = n.advertiseService("Shoot",&Nubot_HWController::ShootControlServive,this);
}

Nubot_HWController::~Nubot_HWController()
{
    try
    {
        Elmo_Process_2(zero2);
        Elmo_Process_1(zero4,0);

        Base_Enable(false);
        Ballhandle_Enable(false);
        #if USE_CURSE
            endwin();
        #endif
    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << std::endl;
    }

}

void Nubot_HWController::Run()
{
    //boost::thread_group g;
    //timer1_.async_wait(strand_.wrap(boost::bind(&Nubot_HWController::Process_10ms, this)));
    //timer2_.async_wait(strand_.wrap(boost::bind(&Nubot_HWController::Process_30ms, this)));
    //timer1_.async_wait((boost::bind(&Nubot_HWController::Process_10ms, this)));
    //timer2_.async_wait((boost::bind(&Nubot_HWController::Process_30ms, this)));
    //g.create_thread(boost::bind(&boost::asio::io_service::run, &io));
    //g.create_thread(boost::bind(&boost::asio::io_service::run, &io));

    //boost::thread t1(boost::bind(&boost::asio::io_service::run, &io));
   // boost::thread t2(boost::bind(&boost::asio::io_service::run, &io));

    ros::spin();
   // t1.join();
   // t2.join();

    //g.join_all();

}

///////////////////////////////////////////////////////
/****************** HKH_201405 ***********************
//  功能: 初始化电位基准读数和量程两个参数
//  说明: 使用带球机构前需要确定电位器的最低和最高位置,
//		  最低位置对应完全没有碰到球的状态,最高位置对应
//		  球完全抱紧的状态.这两个参数取决于机构,比赛前
//		  需要进行标定.
//		  本函数负责执行这个标定功能,在比赛前应该先调
//		  用这个函数来确定上面说的两个参数,得到的参数会
//		  写在main目录下的DribbleParam.txt文件.这个函数
//		  也负责从这个文件读取参数
/*****************************************************/
#define  DribbleDebug 0
bool Nubot_HWController::DribbleParamRead(void)
{
    // 从文件读取之前测定的参数
    ifstream param("DribblePara.txt");
    if(param)
    {
        param>>BallSensor_LBias				// 电位器的基准读数,对应没有球时候的角度
            >>BallSensor_RBias
            >>BallSensor_LStroke			// 电位器最大与最小读数的差值
            >>BallSensor_RStroke;
        param.close();
        LeverPos_SetPoint= 0.7;
        IsBallSensorInited = true;
    }
    else
        IsBallSensorInited = false;
    return IsBallSensorInited;
}
bool cmac_enable=false;
/****************** HKH_201405 ***********************
//	主动带球最底层的控制函数,根据电位器读数和车速进行反馈和前馈控制
//  入口参数:
//		FBRatio : 反馈量的权重,默认=1.0f,置0时相当于Disable反馈输出
//		FFRatio : 前馈量的权重,同上
//  备注:球完全抱紧时
//		原地旋转最大角速率 Wmax=2.0
//		最大侧移速度 Vy=50
********************************************************/
void Nubot_HWController::DribbleController()
{

    if(!IsBallSensorInited)
    {
        // 没初始化跑个屁啊，先标定参数吧～
        DribbleParamCalibrate();
        return;
    }

    // 处理电位器数据
    DribbleGetState();

    if(!BallHandleEnable)
        return;

    const double VelRatio=75/105.0*1.414;   // 滚动轮线速度/球线速度,由机构决定
    const double WheelDiameter=5.2f;        // 主动轮半径,cm
    const double GearRatio=3.47f;           // 主动带球电机齿轮箱减速比
    const double K0 = -VelRatio/(M_PI*WheelDiameter);	// 车体速度cm/s 传递到带球电机转速(圈/秒)

#if 0
    /* 主动带球的闭环PID反馈控制*/
    if(Vx_diff>50*2)
        LeverPos_SetPoint=0.6;
    else if(Vx_diff<=-50*2 || abs(w/(Vx+0.01))>3)
        LeverPos_SetPoint=1.2;
    else
        LeverPos_SetPoint=0.8;
    printf("diff:%.1f, w/x:%.1f",Vx_diff,w/(Vx+0.01));
#endif


    LeverPos_SetPoint= Vx<=0? 0.9 : 0.6;


    LeverPos_ErrL = LeverPos_SetPoint - LeverPos_Left;
    LeverPos_ErrR = LeverPos_SetPoint - LeverPos_Right;
    // 用环形buffer记录历史误差，最新的误差索引总是为2，过去的数据索引依次递减
    e_l.push_back(LeverPos_ErrL);
    e_r.push_back(LeverPos_ErrR);
    printf("error_left:(%.1f) error_right:(%.1f)\n",e_l[2],e_r[2]);

#if 0
    // 增量式PID，注意I参数不能为0，否则算法不起作用
    static double FeedBackVel_Left=0,FeedBackVel_Right=0;
    FeedBackVel_Left += P*(e_l[2]-e_l[1])+I*e_l[2]+D*(e_l[2]-2*e_l[1]+e_l[0]),
    FeedBackVel_Right+= P*(e_r[2]-e_r[1])+I*e_r[2]+D*(e_r[2]-2*e_r[1]+e_r[0]);
#else
    // 绝对式PD
    double FeedBackVel_Left  = e_l[2]*P + (e_l[2]-e_l[1])*D,
            FeedBackVel_Right= e_r[2]*P + (e_r[2]-e_r[1])*D;
#endif

#if 1
    // 异常情况处理:
    // 1.球没有同时接触两只爪爪
    if ( fabs(LeverPos_Diff)>0.3                     // 两爪不平衡
         && (LeverPos_ErrL>0.25 || LeverPos_ErrR>0.25))		// 同时有一个爪的正误差较大
    {
        // 确保碰到球的那只爪不会往外推球
        FeedBackVel_Left  = (FeedBackVel_Left<0) ? 0 : FeedBackVel_Left;
        FeedBackVel_Right = (FeedBackVel_Right<0)? 0 : FeedBackVel_Right;

        // 将没有碰到球的那只爪的速度叠加到另外一只上
        if (LeverPos_Diff>0)
            FeedBackVel_Left += FeedBackVel_Right;    // 球在左边,往左边叠加
        else
            FeedBackVel_Right+= FeedBackVel_Left;     // 同理
    }
#endif
    FeedBackVel_Left  = Limmit(-20,FeedBackVel_Left,20);
    FeedBackVel_Right = Limmit(-20,FeedBackVel_Right,20);

    /* 车速前馈控制,主要是对Vx的补偿,Vy和W的效果不好 */
    double FeedForwardVel_Left=0,FeedForwardVel_Right=0;

#if 0
    /*用小脑模型控制器学习前馈*/
    //if(cmac_enable && LeverPos_Left>0.2 && LeverPos_Right>0.2)
    {
        double input[3]={Vx,LeverPos_Left,LeverPos_Right};
        double output[2],delta[2];

        cmac.Calc(input,output);
        delta[0]=0.4*(FeedBackVel_Left -output[0]);
        delta[1]=0.4*(FeedBackVel_Right-output[1]);
        if(abs(LeverPos_ErrL) < 0.3 && abs(LeverPos_ErrR)<0.3)
            cmac.Train(delta);

        FeedForwardVel_Left =output[0];
        FeedForwardVel_Right=output[1];
    }
#else

    const double Kw = 25*K0;
    // 前馈系数,前进与后退受力不同,故参数不同
    double Kx = Vx>0? Kx_f*K0 : Kx_b*K0;

    FeedForwardVel_Left  = Vx*Kx + w*Kw;
    FeedForwardVel_Right = Vx*Kx - w*Kw;

#endif

#if 1
    // 没带上球的时候停止速度前馈
    if(LeverPos_ErrL>0.3)
        FeedForwardVel_Left = 0;
    if (LeverPos_ErrR>0.3)
        FeedForwardVel_Right = 0;
#endif

    printf("FeedBackVel:(%.1f %.1f) FeedForwardVel:(%.1f %.1f)\n",FeedBackVel_Left,FeedBackVel_Right,FeedForwardVel_Left,FeedForwardVel_Right);

    // 控制器输出:主动带球电机转速(RPM)=反馈输出x反馈权重 + 前馈输出x前馈权重 */
    motor_left = (FeedBackVel_Left  + FeedForwardVel_Left )*60.0*GearRatio;
    motor_right= (FeedBackVel_Right + FeedForwardVel_Right)*60.0*GearRatio;

    // 超过了最高速度elmo会报错
    motor_left = Limmit(-6000,motor_left, 6000);
    motor_right= Limmit(-6000,motor_right,6000);

    BallLockEnable(IsBallLost);

    // reverting the right motor speed
    motor_right=-motor_right;
    Elmo_Process_2(motor_speed);



    printf("Target:%.1f,Left:%.1f,Right:%.1f output:L:%d,R:%d\n",LeverPos_SetPoint,LeverPos_Left,LeverPos_Right,motor_left,motor_right);
#if 0
    ROS_INFO("%s %s(%.1f,%.1f) ",st,IsBallLost? "L":"H",BallDistance_Left,BallDistance_Right);
    ROS_INFO("Vx:%.1f(%.1f) Vy:%.1f w:%.1f\t",Vx,RelVx,Vy,w);
    ROS_INFO("%s(B%.1f(%.1f),F%.1f) %+.1f%+.1f=%+.1f %+.1f%+.1f=%+.1f \n",
        isopenloop? "O":"C",
        FBRatio,BallHoldingDistance,FFRatio,
        FeedBackVel_Left,FeedForwardVel_Left*FFRatio,activeBallVleft,
        FeedBackVel_Right,FeedForwardVel_Right*FFRatio,activeBallVright);
#endif
}


void Nubot_HWController::ParamChanged(nubot_HWController::controllerConfig &config, uint32_t level)
{
    cmac_enable=config.Use_CMAC;
    P = config.FeedBack_P;
    D = config.FeedBack_D;
    I = config.FeedBack_I;
    Kx_f=config.Kx_f;
    Kx_b=config.Kx_b;
    LeverPos_SetPoint = config.Target_Position;
}

const double WHEEL_DISTANCE=20.3;
const double MOTOR_GEAR_RATIO=49.0/4.0;
const double WHEEL_DIAMETER=12.0;
const double VEL_TO_RPM_RATIO=MOTOR_GEAR_RATIO*60.0/(M_PI*WHEEL_DIAMETER);

void Nubot_HWController::Timer1_Process(const ros::TimerEvent &e)
{
    //clear();
    ros::Duration time_err=e.current_real-e.current_expected;
    printf("Time error:%.2fms,Last duration:%.2fms\n",time_err.toSec()*1000,(e.profile.last_duration).toSec()*1000);

    printf("Vx:%.1f,Vy:%.1f,w:%.1f\n",Vx,Vy,w);

    static trigger<3> t;
    if(t())
    {
        BaseController();

        // 发送里程计信息
        if (OdeInfo_pub->trylock())
        {
            OdeInfo_pub->msg_.header.stamp = ros::Time::now();
            OdeInfo_pub->msg_.w =Real_w;
            OdeInfo_pub->msg_.Vx=Real_Vx;
            OdeInfo_pub->msg_.Vy=Real_Vy;
            OdeInfo_pub->unlockAndPublish();
        }
    }

    DribbleController();
    // 发送状态信息
    if (DebugInfo_pub->trylock())
    {
        DebugInfo_pub->msg_.d1 =LeverPos_Left;//LeverPos_Left;
        DebugInfo_pub->msg_.d2 =LeverPos_Right;
        DebugInfo_pub->unlockAndPublish();
    }

#if 0
    static trigger<500> t2;
    static bool en=false;
    if(t2())
        en=!en,BallLockEnable(en);
        //LeverPos_SetPoint = LeverPos_SetPoint==0.5? 0.7 : 0.5;
#endif

   //refresh();
}

/* int64 req.enabble = 0, unable
 * int64 req.enabble = 1, enable
 *
 * int64 req.strength =  the shoot power
 *
 * int64 res.response = 0, shoot state: faild
 * int64 res.response = 1, shoot state: success
 */
bool Nubot_HWController::ShootControlServive(nubot_common::Shoot::Request  &req,
                         nubot_common::Shoot::Response &res)
{
    ShootPower = req.strength;
    //Process_Shoot();
    boost::thread t1(boost::bind(&Nubot_HWController::Process_Shoot, this));

    res.ShootIsDone = ShootDone? 1:0;

    return true;
}

/* int64 req.enable = 0, unable
 * int64 req.enable = 1, enable
 * int64 res.response = 0, dribble state: no ball
 * int64 res.response = 1, dribble state: holding ball
 */
bool Nubot_HWController::BallHandleControlService(nubot_common::BallHandle::Request  &req,
                              nubot_common::BallHandle::Response &res)
{
    if(BallHandleEnable!=req.enable)
        Ballhandle_Enable(req.enable);

    BallHandleEnable=req.enable;

    res.BallIsHolding = BallSensor_IsHolding;

    return true;
}

const double LIMITEDRPM=12000;
void Nubot_HWController::BaseController(/*const ros::TimerEvent &event*/)
{
    int32 v[4], vmax,Real_v[7];

    v[0]= ( -0.707*Vy + 0.707*Vx - w*WHEEL_DISTANCE)*VEL_TO_RPM_RATIO;
    v[1]= (  0.707*(Vx + Vy)     - w*WHEEL_DISTANCE)*VEL_TO_RPM_RATIO;
    v[2]= (  0.707*(-Vx + Vy)    - w*WHEEL_DISTANCE)*VEL_TO_RPM_RATIO;
    v[3]= ( -0.707*Vx - 0.707*Vy - w*WHEEL_DISTANCE)*VEL_TO_RPM_RATIO;

    vmax = abs(v[0]);
    for(int i  = 1; i <  4; i++)
           if(abs(v[i]) >  vmax)
              vmax = abs(v[i]);

    if(vmax > LIMITEDRPM)
        for(int i  = 0 ; i <  4; i++)
             v[i] = v[i]*LIMITEDRPM/vmax;

    Elmo_Process_1(v,Real_v);
    printf("M1: %d  M2: %d  M3: %d  M4: %d  M5: %d  M6: %d  M7: %d\n",Real_v[0],Real_v[1],Real_v[2],Real_v[3],Real_v[4],Real_v[5],Real_v[6]);

    Real_w  = (-Real_v[1] - Real_v[3])/(2*WHEEL_DISTANCE*VEL_TO_RPM_RATIO);
    Real_Vx = (Real_v[0]+Real_v[1]-Real_v[2]-Real_v[3])/(2*1.414*VEL_TO_RPM_RATIO);
    Real_Vy = (Real_v[1]+Real_v[2]-Real_v[0]-Real_v[3])/(2*1.414*VEL_TO_RPM_RATIO);

}

void Nubot_HWController::Read_VelCmd(const nubot_common::VelCmd::ConstPtr& cmd)
{
    Vx=cmd->Vx;
    Vy=cmd->Vy;
    w =cmd->w;

    Vxs.push_back(Vx);

    // 判断车体运动状态
    Vx_diff = Vxs[3]+Vxs[2]-(Vxs[1]+Vxs[0]);
}








/****************** HKH_201405 ***********************
//  功能: 标定主动带球电位器最高和最低读数
//  说明: 本函数为non-blocking函数，你需要按一定周期调用本函数，
//     当函数返回true时表示标定已完成
/*****************************************************/
bool Nubot_HWController::DribbleParamCalibrate(void)
{
    static int state=0;
    static int cnt=0;
    static float sum_l=0,sum_r=0;

    const int NUM=120;

    unsigned int BallHandlingSensor_Left,BallHandlingSensor_Right;
    try{
        // 获得电位器读数
       BallHandlingSensor_Left = in_EL3064->invalue1;
       BallHandlingSensor_Right= in_EL3064->invalue2;
    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << std::endl;
    }
#if DribbleDebug
    //static ofstream dbg("drible.txt");
   // static trigger<10> t;
   // if(t())
        ROS_INFO("L:%d	R:%d",in_EL3064->invalue1,in_EL3064->invalue2);
#endif

    IsBallSensorInited = false;
    switch (state)
    {
    case 0:// 统计电位器的基准电压
        if(cnt<NUM)
        {
            if ( BallHandlingSensor_Left != 0 )
            {
                sum_l += (float)BallHandlingSensor_Left;
                sum_r += (float)BallHandlingSensor_Right;
                cnt++;
            }
        }

        //已经采集了足够的数据
        else
        {
            // 对30个数据求平均来实现滤波
            BallSensor_LBias = sum_l/NUM;					// 基准电压
            BallSensor_RBias = sum_r/NUM;
#if DribbleDebug
            ROS_INFO("LBias:%.3f	RBias:%.3f",BallSensor_LBias,BallSensor_RBias);
#endif
            //跳转到下个状态
            state = 1;
            cnt = 0;
            sum_l = sum_r = 0.0f;
        }
        motor_left=motor_right=0;
        break;

    case 1:// 驱动电机，记录完全持球时的最高电压
        if(cnt<(NUM+10))
        {
            // 还没抓到球，使劲抓球
            if (  BallHandlingSensor_Left > BallSensor_LBias+20
                &&BallHandlingSensor_Right > BallSensor_RBias+20)
            {
                if (cnt>9)
                {
                    sum_l += (float)BallHandlingSensor_Left;
                    sum_r += (float)BallHandlingSensor_Right;
                }
                cnt++;
                motor_left = 600; motor_right= -motor_left;
            }
             // 已经抓到球，减慢滚球速度
            else
            {
                motor_left = 300; motor_right= -motor_left;
            }
        }
        else  // 已经采集了足够的数据
        {
            // 对数据求平均来滤波
            float BallSensor_LMax = sum_l/NUM;
            float BallSensor_RMax = sum_r/NUM;

            if (BallSensor_LMax>BallSensor_LBias && BallSensor_RMax>BallSensor_RBias)
            {
                BallSensor_LStroke = BallSensor_LMax - BallSensor_LBias; // 电压范围=max-min
                BallSensor_RStroke = BallSensor_RMax - BallSensor_RBias;
                IsBallSensorInited = true;
#if DribbleDebug
                ROS_INFO("LStroke:%.3f	RStroke:%.3f",BallSensor_LStroke,BallSensor_RStroke);
#endif
                ofstream out("DribblePara.txt");
                out<<BallSensor_LBias<<' '
                    <<BallSensor_RBias<<' '
                    <<BallSensor_LStroke<<' '
                    <<BallSensor_RStroke;
                out.close();
                LeverPos_SetPoint=0.7;
                ROS_INFO("Calibration done!");
                motor_left=motor_right=0;
            }
            else  //采到的数据不合格，重来
            {
                cnt = 0;
                sum_l = sum_r = 0.0f;
            }
        }
        break;
    default:
        break;
    }

    Elmo_Process_2(motor_speed);

    return IsBallSensorInited;
}


/****************** HKH_201405 ***********************
//  功能: 通过电位器读数获得带球机构的状态
//  说明: 本函数对原始AD读数进行归一化和滤波,并更新和
//		  返回相关的状态变量
/*****************************************************/
bool Nubot_HWController::DribbleGetState(void)
{
    // 获得电位器读数
    int BallSensor_Left,BallSensor_Right;
    try
    {
        BallSensor_Left = in_EL3064->invalue1 -BallSensor_LBias;
        BallSensor_Right= in_EL3064->invalue2 -BallSensor_RBias;
    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << std::endl;
    }

    printf("L0:%d,	R0:%d\n",
           BallSensor_Left+(int)BallSensor_LBias,
           BallSensor_Right+(int)BallSensor_RBias);

    // 将电位器读数归一化,0对应完全没碰到球,1对应球完全进来了
    LeverPos_Left  = BallSensor_Left /BallSensor_LStroke;
    LeverPos_Right = BallSensor_Right/BallSensor_RStroke;

#if 0
    // 取最近三次的平均值
    static DPoint filter[3]={DPoint(0,0),DPoint(0,0),DPoint(0,0)};
    static int idx = 0;
    idx = (++idx>2)? 0 : idx;
    filter[idx].x = BallDistance_Left;
    filter[idx].y = BallDistance_Right;
    DPoint result = (filter[0] + filter[1] + filter[2])/3;
    BallDistance_Left = result.x;
    BallDistance_Right= result.y;
#endif

    // 限幅,防止偶发的小于0或大于1的情况
    LeverPos_Left = Limmit(0.0f, LeverPos_Left,  1.0f);
    LeverPos_Right= Limmit(0.0f, LeverPos_Right, 1.0f);

    // 左右球距的偏差
    LeverPos_Diff = LeverPos_Left - LeverPos_Right;

    // 左右球距均连续n+1个周期大于期望球距的%80时才认为已经带上球,防止状态切换时震荡
    static int HoldingCnt = 0;
    if(LeverPos_Left > 0.3 && LeverPos_Right > 0.3)
        HoldingCnt++;
    else
        HoldingCnt=0;
    BallSensor_IsHolding = (HoldingCnt>4);

    IsBallLost = LeverPos_Left<0.2 && LeverPos_Right<0.2;

    return BallSensor_IsHolding;
}


void Nubot_HWController::BallLockEnable(bool on)
{
    if(on)
        EL2008_set(BallLock_IO);
    else
        EL2008_clear(BallLock_IO);
}


#define  Delay2IGBT     100   //继电器导通到IGBT导通控制时间间隔,一次1ms,共100ms
#define  ShotFinishTime 500  //射门完成周期,一次1ms,共500ms
int kicktime=0;
void Nubot_HWController::Shoot_Control(uint8 *IsKick, uint16 ShootPower)
{
    if(*IsKick==1 && kicktime<ShotFinishTime) //
    {
        if(kicktime<Delay2IGBT)
        {
            EL2008_set(Relay_IO);
        }
        else if(kicktime<Delay2IGBT+ShootPower)
        {
            EL2008_set(Relay_IO);
            EL2008_set(IGBT_IO);
        }
        else
        {
            EL2008_clear(Relay_IO);
            EL2008_clear(IGBT_IO);
        }
        kicktime++;
    }
    else
    {
        EL2008_clear(Relay_IO);
        EL2008_clear(IGBT_IO);
        *IsKick=0;
        kicktime=0;
    }
}

void Nubot_HWController::Process_Shoot()
{
    boost::asio::deadline_timer timer(io);

    ptime start= microsec_clock::local_time();
    ptime now  = microsec_clock::local_time();

    time_duration diff = now - start;
    while(diff < millisec(Delay2IGBT) )
    {
        EL2008_set(Relay_IO);


        //时间还没够
        time_duration dt=min(millisec(20), millisec(Delay2IGBT)-diff);
        usleep(dt.total_microseconds());
        //timer3_.expires_from_now(dt);
        //timer3_.wait(boost::bind(&Nubot_HWController::Process_Shoot, this));
        diff = microsec_clock::local_time() - start;
        std::cout<<diff.total_milliseconds()<<"-OpenRelay"<<endl;
    }

    while(diff < millisec(Delay2IGBT+ShootPower) )
    {
        EL2008_set(Relay_IO);
        EL2008_set(IGBT_IO);

        //时间还没够
        time_duration dt=min(millisec(20), millisec(Delay2IGBT+ShootPower)-diff);
        usleep(dt.total_microseconds());
        //timer3_.expires_from_now(dt);
        //timer3_.wait(boost::bind(&Nubot_HWController::Process_Shoot, this));
        diff = microsec_clock::local_time() - start;
        std::cout<<diff.total_milliseconds()<<"-OpenIGBT"<<endl;
    }

    EL2008_clear(Relay_IO);
    EL2008_clear(IGBT_IO);
    ShootDone = true;
    std::cout<<"-Done"<<endl;



#if 0
    static ptime start;

    ptime now = microsec_clock::local_time();
    time_duration diff = now - start;
    switch(ShootState)
    {
    // 启动时钟顺序
    case 0:
        start=now;
        diff =millisec(0);
        ShootState=1;
        ShootDone = false;

    //  Relay预启动
    case 1:
        //EL2008_set(Relay_IO);
        if(diff < millisec(Delay2IGBT) )
        {
            //时间还没够
            time_duration dt=min(millisec(20), millisec(Delay2IGBT)-diff);
            timer3_.expires_from_now(dt);
            timer3_.wait(boost::bind(&Nubot_HWController::Process_Shoot, this));
        }
        else
            ShootState=2;
        break;

    // 打开IGBT，电磁铁使劲
    case 2:
        //EL2008_set(Relay_IO);
        //EL2008_set(IGBT_IO);
        if(diff < millisec(Delay2IGBT+ShootPower) )
        {
            //时间还没够
            time_duration dt=min(millisec(20), millisec(Delay2IGBT+ShootPower)-diff);
            timer3_.expires_from_now(dt);
            timer3_.wait(boost::bind(&Nubot_HWController::Process_Shoot, this));
        }
        else
            ShootState=3;
        break;

    // 打完收工
    case 3:
        //EL2008_clear(Relay_IO);
        //EL2008_clear(IGBT_IO);
        ShootDone = true;
        ShootState=0;
        break;

    default:
        ShootState=0;
        break;
    }
    std::cout<<diff.total_milliseconds()<<" state:"<<ShootState<<endl;
#endif
}


void Nubot_HWController::Process_10ms()
{
    static ros::Time last=ros::Time::now();
    ros::Duration dt=ros::Time::now()-last;
    last=ros::Time::now();
    double dus=1000-dt.toNSec()/1000.0;


    if(ros::ok())
    {
        timer1_.expires_at(timer1_.expires_at() + boost::posix_time::millisec(10));
        timer1_.async_wait(strand_.wrap(boost::bind(&Nubot_HWController::Process_10ms, this)));

        #if USE_CURSE
            clear();
        #endif
        printf("dt:%10.3fus\n",dus/*diff.total_microseconds()*/);
        DribbleController();
        // 发送状态信息
        if (DebugInfo_pub->trylock())
        {
            DebugInfo_pub->msg_.d1 =LeverPos_Left;//LeverPos_Left;
            DebugInfo_pub->msg_.d2 =LeverPos_Right;
            DebugInfo_pub->unlockAndPublish();
        }
        #if USE_CURSE
        refresh();
        #endif
    }

    //static trigger<100> t;
    //if(t())
}

void Nubot_HWController::Process_30ms()
{
    if(ros::ok())
    {
        //clear();
        timer2_.expires_at(timer2_.expires_at() + boost::posix_time::millisec(30));
        timer2_.async_wait(strand_.wrap(boost::bind(&Nubot_HWController::Process_30ms, this)));


        BaseController();
        printf("Vx:%.1f,Vy:%.1f,w:%.1f\n",Vx,Vy,w);
        printf("RVx:%.1f,RVy:%.1f,Rw:%.1f\n",Real_Vx,Real_Vy,Real_w);

        // 发送里程计信息
        if (OdeInfo_pub->trylock())
        {
            OdeInfo_pub->msg_.header.stamp = ros::Time::now();
            OdeInfo_pub->msg_.w =Real_w;
            OdeInfo_pub->msg_.Vx=Real_Vx;
            OdeInfo_pub->msg_.Vy=Real_Vy;
            OdeInfo_pub->unlockAndPublish();
        }
        //refresh();
    }
}


int main(int argc,char** argv)
{
    ros::init(argc,argv,"Nubot_HWController");

    Nubot_HWController controller;

    controller.Run();


    return 0;
}







