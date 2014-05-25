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

#include "nubot_standalone/VelCmd.h"
#include <nubot_standalone/BallHandle.h>
#include <nubot_standalone/Shoot.h>


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

	double deadzone_;

	void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
};

TeleopNubot::TeleopNubot()
    :Vx(0),Vy(0),w(0)
{
    vel_pub = n.advertise<nubot_standalone::VelCmd>("Motion_Velcmd", 1);
	joy_sub = n.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopNubot::joyCallback, this);
    ballhandle_client_ =  n.serviceClient<nubot_standalone::BallHandle>("BallHandle");
    shoot_client_ = n.serviceClient<nubot_standalone::Shoot>("Shoot");
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
    nubot_standalone::VelCmd cmd;
    cmd.Vx = joy->axes[idx_X]*300;
    cmd.Vy = joy->axes[idx_Y]*0;
    cmd.w  = joy->axes[idx_w]*M_PI;
    vel_pub.publish(cmd);

    static int A_last = 0;
    if(A_last==0 && joy->buttons[Button_A]==1)
    {
        nubot_standalone::Shoot s;
        s.request.strength=4;
        shoot_client_.call(s);
    }

    static int B_last = 0;
    static bool BallHandleEnable = false;
    if(B_last==0 && joy->buttons[Button_B]==1)
    {
        BallHandleEnable=!BallHandleEnable;
        nubot_standalone::BallHandle b;
        b.request.enable=BallHandleEnable;
        ballhandle_client_.call(b);
    }

    A_last = joy->buttons[Button_A];
    B_last = joy->buttons[Button_B];
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
		if(execlp("rosrun", "rosrun", "joy", "joy_node", (char *)0) <0)
			ROS_WARN("Process Joy not found!");

		// 正常情况下exec函数不会返回
		return(-1);
	}
#endif


    ros::spin();
	return (0);
}


