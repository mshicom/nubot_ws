#include <iostream>
#include <fstream>
#include <sstream>

#include <opencv2/opencv.hpp>
#include "nubot/core/core.hpp"
#include "nubot/omni_vision/transfer.h"
#include "nubot/omni_vision/scanpoints.h"
#include "nubot/omni_vision/whites.h"
#include "nubot/omni_vision/optimise.h"
#include "nubot/omni_vision/glocalization.h"
#include "nubot/omni_vision/robotinformation.h"
#include "nubot/omni_vision/odometry.h"
#include "nubot/omni_vision/localization.h"
#include "nubot/omni_vision/obstacles.h"
#include "nubot/omni_vision/ballfinder.h"

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/String.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <std_msgs/Header.h>
#include <image_transport/image_transport.h>
#include <image_transport/camera_subscriber.h>

#include <nubot_standalone/MotorInfo.h>
#include <nubot_standalone/RobotInfo.h>
#include <nubot_standalone/BallInfo.h>
#include <nubot_standalone/ObstaclesInfo.h>
#include <nubot_standalone/OminiVisionInfo.h>

using namespace cv;
using namespace nubot;
namespace encodings=sensor_msgs::image_encodings;
namespace nubot {


class Omni_Vision
{

private:

    Omni_Image       * imginfo_;
    Transfer         * tranfer_;
    ScanPoints       * scanpts_;
    Whites           * whites_;
    Optimise         * optimise_;
    Globallocalization * glocation_;
    RobotInformation * robotinfo_;
    Odometry         * odometry_;
    Localization     * location_;
    Obstacles        * obstacles_;
    BallFinder       * ball_finder_;

    image_transport::Subscriber img_sub_;
    ros::Publisher  ballinfo_pub_;
    ros::Publisher  robotinfo_pub_;
    ros::Publisher  obstaclesinfo_pub_;
    ros::Publisher  omin_vision_pub_;
    ros::Subscriber motor_info_sub_;

    nubot_standalone::BallInfo        ball_info_;
    nubot_standalone::RobotInfo       robot_info_;
    nubot_standalone::ObstaclesInfo   obstacles_info_;
    nubot_standalone::OminiVisionInfo omin_vision_info_;

public:

Omni_Vision()
{
    ROS_INFO("initialize omni_vision process");
    imginfo_     = new Omni_Image("/home/nubot2/ros_workspace/src/nubot_standalone/omni_vision/calibration/ROI.xml");
    tranfer_     = new Transfer("/home/nubot2/ros_workspace/src/nubot_standalone/omni_vision/calibration/mirror_calib.xml",*imginfo_);
    scanpts_     = new ScanPoints(*imginfo_);
    whites_      = new Whites(*scanpts_,*tranfer_);
    optimise_    = new Optimise("/home/nubot2/ros_workspace/src/nubot_standalone/omni_vision/calibration/errortable.txt",
                                "/home/nubot2/ros_workspace/src/nubot_standalone/omni_vision/calibration/difftable.txt",*tranfer_);
    glocation_   = new Globallocalization(*optimise_);
    robotinfo_   = new RobotInformation();
    odometry_    = new Odometry();
    location_    = new Localization(*optimise_,*odometry_);
    obstacles_   = new Obstacles(*scanpts_,*tranfer_);
    ball_finder_ = new BallFinder("/home/nubot2/ros_workspace/src/nubot_standalone/omni_vision/calibration/CTable.dat",*tranfer_);

    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    img_sub_= it.subscribe("/camera/image_raw", 1, &Omni_Vision::imageCallback,this);
    ros::NodeHandle local_nh;
    motor_info_sub_ = local_nh.subscribe("odometry/motorinfo", 1, &Omni_Vision::odometryupdate,this);

    ros::NodeHandle node;
    ballinfo_pub_      = node.advertise<nubot_standalone::BallInfo>("/omnivision/ballinfo",1);
    robotinfo_pub_     = node.advertise<nubot_standalone::RobotInfo>("/omnivision/robotinfo",1);
    obstaclesinfo_pub_ = node.advertise<nubot_standalone::ObstaclesInfo>("/omnivision/obstcalesinfo",1);
    omin_vision_pub_   = node.advertise<nubot_standalone::OminiVisionInfo>("/omnivision/OminiVisionInfo",1);
}
~Omni_Vision()
{


}


private:

void
process()
{
    //ROS_INFO("localization process");
    scanpts_->process();
    whites_->process();
    //whites_->showWhitePoints();
    if(whites_->img_white_.size()<=0)
    {
       ROS_WARN("don't detect the white points");
       return ;
    }
    if(robotinfo_->isglobal_)
       robotinfo_->isglobal_=glocation_->process(whites_->weights_,whites_->robot_white_,robotinfo_->location_,robotinfo_->angle_);
    else
       location_->process(whites_->weights_,whites_->robot_white_,robotinfo_->location_,robotinfo_->angle_);
    obstacles_->process();
    obstacles_->show();

    ball_info_.pos_known=ball_finder_->Process(imginfo_->yuv_image_,robotinfo_->location_,robotinfo_->angle_);
    //cv::imshow("segment",ball_finder_->color_segment_->segment_result_*75);
    //cv::waitKey(10);
    robot_info_.header.stamp = ros::Time::now();
    robotinfo_->realvtrans_       = odometry_->get_real_velocity();
    robotinfo_->worldvtrans_      = odometry_->get_world_velocity();
    robotinfo_->angular_velocity_ = odometry_->get_angular_velocity();
    robot_info_.header.seq++;
    robot_info_.heading.theta=robotinfo_->angle_.radian_;
    robot_info_.pos.x    = robotinfo_->location_.x_;
    robot_info_.pos.y    = robotinfo_->location_.y_;
    robot_info_.vtrans.x = robotinfo_->worldvtrans_.x_;
    robot_info_.vtrans.y = robotinfo_->worldvtrans_.y_;
    robot_info_.vrot = robotinfo_->angular_velocity_.radian_;

    ROS_INFO("x: %f y: %f vx: %f  vy: %f angle: %f ",robotinfo_->location_.x_,
             robotinfo_->location_.y_,robotinfo_->worldvtrans_.x_,robotinfo_->worldvtrans_.y_,robotinfo_->angle_.radian_);

    obstacles_info_.header.stamp= ros::Time::now();
    obstacles_info_.header.seq++;
    std::vector<DPoint> obscount;
    int obsnums=obstacles_->obstacles_.size();
    int obs_nums_infiled(0);
    if(obsnums>0)
    {
        obscount=tranfer_->worldcoordinates(obstacles_->obstacles_,robotinfo_->location_,robotinfo_->angle_);
        for(size_t j=0;j < obsnums ;j++)
        {
            if(std::abs(obscount[j].x_)<900&&std::abs(obscount[j].y_<600)&& obs_nums_infiled<10)
            {
               obstacles_info_.pos[obs_nums_infiled].x=obscount[j].x_;
               obstacles_info_.pos[obs_nums_infiled].y=obscount[j].y_;
               obs_nums_infiled++;
        //       ROS_INFO("x: %f y: %f",obstacles_info_.pos[obs_nums_infiled-1].x,
        //                  obstacles_info_.pos[obs_nums_infiled-1].y);
             }
            obstacles_info_.length=obs_nums_infiled;
        }
    }
    else
        obstacles_info_.length=obs_nums_infiled;

    ball_info_.header.stamp =  ros::Time::now();
    ball_info_.header.seq++;
    DPoint ball_pos = ball_finder_->get_ball_global_loc();
    DPoint ball_vec = ball_finder_->get_ball_velocity();
    PPoint ball_real_loc=ball_finder_->get_ball_real_loc();
    ball_info_.pos.x = ball_pos.x_;
    ball_info_.pos.y = ball_pos.y_;
    ball_info_.velocity.x=ball_vec.x_;
    ball_info_.velocity.y=ball_vec.y_;
    ball_info_.real_pos.angle=ball_real_loc.angle_.radian_;
    ball_info_.real_pos.radius=ball_real_loc.radius_;
    ball_info_.velocity_known = ball_finder_->is_velocity_known() ;
    ROS_INFO("x: %f y: %f vx: %f  vy: %f ",ball_info_.pos.x,
               ball_info_.pos.y,ball_info_.velocity.x,ball_info_.velocity.y);

    omin_vision_info_.header.stamp = ros::Time::now();
    omin_vision_info_.header.seq++;
    omin_vision_info_.ballinfo=ball_info_;
    omin_vision_info_.obstacleinfo=obstacles_info_;
    omin_vision_info_.robotinfo=robot_info_;
    omin_vision_pub_.publish(omin_vision_info_);


}

 void
 imageCallback(const sensor_msgs::ImageConstPtr& _image_msg)
 {
 //   ROS_INFO("start omni_vision imageCallback");
    cv_bridge::CvImageConstPtr cv_ptr;
    cv_ptr=cv_bridge::toCvShare(_image_msg,encodings::BGR8);
    Mat orignalimage=cv_ptr->image;
    Mat ROI_Image=orignalimage;;
    bool isthreechannels=imginfo_->set_bgr_image(ROI_Image);
    if(!isthreechannels)
    {
       ROS_WARN("the image doesn't have three channels");
       return ;
     }
    imginfo_->bgr2yuv();
    process();
}

void
odometryupdate(const nubot_standalone::MotorInfo & _motorinfo_msg)
{
    static std::vector<int> motor_data(nubot::MOTOR_NUMS_CONST,0);
    static ros::Time time_before=ros::Time::now();
    ros::Time time_update = _motorinfo_msg.header.stamp;
    for(int i=0; i < MOTOR_NUMS_CONST; i++)
        motor_data[i]=(int)_motorinfo_msg.motordata[i];
    ros::Duration duration= time_update-time_before;
    time_before = time_update ;
    double secs=duration.toSec();
  //  ROS_INFO("m1: %d m2: %d m3: %d m4: %d", motor_data[0],motor_data[1],motor_data[2],motor_data[3]);
    odometry_->process(motor_data,secs);
    DPoint2d world_pt   = odometry_->get_world_locaton();
    Angle    delta_ang  = odometry_->get_delta_angle();
    ROS_INFO("odo_x: %f odo_y: %f odo_ang: %f", world_pt.x_,world_pt.y_,delta_ang.radian_);
}

};


}
// end of namespace
int main(int argc, char **argv)
{ 
	 ros::init(argc,argv,"omnivision");
     ros::Time::init();
     ROS_INFO("start omni_vision process");
     nubot::Omni_Vision vision_process;
	 ros::spin();
	 return 0;
}
