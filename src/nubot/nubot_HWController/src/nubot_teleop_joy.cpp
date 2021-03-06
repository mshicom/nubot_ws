/*
 * nubot_teleop_key.cpp
 *
 *  Created on: 2012-10-11
 *      Author: hkh
 */
#include <ros/ros.h>
#include <stdio.h>
#include <pthread.h>

#include <math.h>

#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

#include <nubot_common/VelCmd.h>
#include <nubot_common/OminiVisionInfo.h>
#include <nubot_common/BallHandle.h>
#include <nubot_common/Shoot.h>


class TeleopNubot
{
public:
	TeleopNubot();
	ros::NodeHandle n;
    ros::ServiceClient ballhandle_client_;
    ros::ServiceClient shoot_client_;

    double Vx,Vy,w;

private:
	ros::Publisher vel_pub;
    ros::Subscriber joy_sub;
    ros::Subscriber ball_sub;

    nubot_common::VelCmd cmd;
    bool CatchEnable;
    double ball_angle;
	double deadzone_;

	void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
    void ballCallback(const nubot_common::OminiVisionInfo::ConstPtr& ball);
};

TeleopNubot::TeleopNubot()
    :Vx(0),Vy(0),w(0),ball_angle(0),CatchEnable(false)
{
    vel_pub = n.advertise<nubot_common::VelCmd>("/nubotcotrol/velcmd", 10);
    joy_sub = n.subscribe<sensor_msgs::Joy>("joy", 2, &TeleopNubot::joyCallback, this);
    ball_sub= n.subscribe<nubot_common::OminiVisionInfo>("/omnivision/OminiVisionInfo", 2, &TeleopNubot::ballCallback, this);
    ballhandle_client_ =  n.serviceClient<nubot_common::BallHandle>("BallHandle");
    shoot_client_ = n.serviceClient<nubot_common::Shoot>("Shoot");
}

// 手柄消息回调函数，在ros::spin()里执行
#define idx_X 1
#define idx_Y 0
#define idx_w 3

#define Button_A    0
#define Button_B    1
#define Button_X    2
#define Button_Y    3

void TeleopNubot::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{

    if(joy->buttons[Button_A]==1)
    {
        nubot_common::Shoot s;
        s.request.strength=4;
        shoot_client_.call(s);
    }

    static bool BallHandleEnable = false;
    if(joy->buttons[Button_B]==1)
    {
        BallHandleEnable=!BallHandleEnable;
        nubot_common::BallHandle b;
        b.request.enable=BallHandleEnable;
        ballhandle_client_.call(b);
    }

    /*速度指令*/
    cmd.Vx = joy->axes[idx_X]*300;
    cmd.Vy = joy->axes[idx_Y]*0;

    CatchEnable = joy->buttons[Button_X];
    // 朝球转
    if(!CatchEnable)
        cmd.w  = joy->axes[idx_w]*M_PI;

    vel_pub.publish(cmd);
}

void TeleopNubot::ballCallback(const nubot_common::OminiVisionInfo::ConstPtr& ball)
{
    ball_angle = ball->ballinfo.real_pos.angle;
    ROS_INFO("Angle:%.2f",ball_angle/M_PI*180.0);

    if(CatchEnable)
    {
        cmd.w  = ball_angle;
        vel_pub.publish(cmd);
    }
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "nubot_teleop_joy");
	TeleopNubot teleop_nubot;

#if 0
	// 调用joy_node进程,驱动手柄
	pid_t pid = vfork();
	if(pid==0)
	{
        if(execlp("rosrun", "rosrun", "joy", "joy_node", "_deadzone:=0.13", (char *)0) <0)
			ROS_WARN("Process Joy not found!");

		// 正常情况下exec函数不会返回
		return(-1);
	}
#endif

    ros::spin();

	return (0);
}


