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
#include "nubot/omni_vision/colorsegment.h"
#include "nubot/omni_vision/ballfinder.h"

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/String.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <std_msgs/Header.h>
#include <image_transport/image_transport.h>
#include <image_transport/camera_subscriber.h>

#include <nubot_common/VelCmd.h>
#include <nubot_common/RobotInfo.h>
#include <nubot_common/BallInfo.h>
#include <nubot_common/ObstaclesInfo.h>
#include <nubot_common/OminiVisionInfo.h>
#include <omni_vision/OmniVisionConfig.h>>
#include <dynamic_reconfigure/server.h>

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
    ColorSegment     * colorsegment_;
    FieldInformation field_info_;

    image_transport::Subscriber img_sub_;
    ros::Subscriber motor_info_sub_;

    ros::Publisher  ballinfo_pub_;
    ros::Publisher  robotinfo_pub_;
    ros::Publisher  obstaclesinfo_pub_;
    ros::Publisher  omin_vision_pub_;


    nubot_common::BallInfo        ball_info_;
    nubot_common::RobotInfo       robot_info_;
    nubot_common::ObstaclesInfo   obstacles_info_;
    nubot_common::OminiVisionInfo omin_vision_info_;
    dynamic_reconfigure::Server<omni_vision::OmniVisionConfig> reconfigureServer_;

    bool is_show_ball_;
    bool is_show_whites_;
    bool is_show_obstacles_;
    bool is_show_scan_points;



public:

Omni_Vision(int argc, char **argv)
{
    std::string calibration_path="/home/nubot2/nubot_ws/src/nubot/omni_vision/calibration";
    if(argc>1)
        calibration_path=argv[1];
    ROS_INFO("initialize the omni_vision  process");
    imginfo_     = new Omni_Image(calibration_path+"/ROI.xml");
    tranfer_     = new Transfer(calibration_path+"/mirror_calib.xml",*imginfo_);
    scanpts_     = new ScanPoints(*imginfo_);
    whites_      = new Whites(*scanpts_,*tranfer_);
    optimise_    = new Optimise(calibration_path+"/errortable.txt",
                                calibration_path+"/difftable.txt",
                                *tranfer_);
    glocation_   = new Globallocalization(*optimise_);
    robotinfo_   = new RobotInformation();
    odometry_    = new Odometry();
    location_    = new Localization(*optimise_);
    obstacles_   = new Obstacles(*scanpts_,*tranfer_);
    ball_finder_ = new BallFinder(*tranfer_);
    colorsegment_= new ColorSegment(calibration_path+"/CTable.dat");

    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    img_sub_= it.subscribe("/camera/image_raw", 1, &Omni_Vision::imageCallback,this);
    ros::NodeHandle local_nh;
    motor_info_sub_ = local_nh.subscribe("/omni_vision/odoinfo", 0, &Omni_Vision::odometryupdate,this);

    ros::NodeHandle node;
    ballinfo_pub_      = node.advertise<nubot_common::BallInfo>("/omnivision/ballinfo",1);
    robotinfo_pub_     = node.advertise<nubot_common::RobotInfo>("/omnivision/robotinfo",1);
    obstaclesinfo_pub_ = node.advertise<nubot_common::ObstaclesInfo>("/omnivision/obstcalesinfo",1);
    omin_vision_pub_   = node.advertise<nubot_common::OminiVisionInfo>("/omnivision/OminiVisionInfo",1);
    reconfigureServer_.setCallback(boost::bind(&Omni_Vision::configure, this, _1, _2));

    is_show_ball_=false;
    is_show_obstacles_=false;
    is_show_whites_=false;
    is_show_scan_points=false;
}
~Omni_Vision()
{


}

private:

void
configure(const omni_vision::OmniVisionConfig & config, uint32_t level)
{
  is_show_ball_       = config.ball_show;
  is_show_whites_     = config.whites_show;
  is_show_obstacles_  = config.obstacles_show;
  is_show_scan_points = config.scanpoints_show;
}

void
publish()
{

    robot_info_.header.stamp = ros::Time::now();
    robot_info_.header.seq++;
    robot_info_.heading.theta = robotinfo_->angle_.radian_;
    robot_info_.pos.x         = robotinfo_->location_.x_;
    robot_info_.pos.y         = robotinfo_->location_.y_;
    robot_info_.vtrans.x      = robotinfo_->worldvtrans_.x_;
    robot_info_.vtrans.y      = robotinfo_->worldvtrans_.y_;
    robot_info_.vrot          = robotinfo_->angular_velocity_;
    ROS_INFO("x: %f y: %f vx: %f  vy: %f angle: %f ",robotinfo_->location_.x_,
             robotinfo_->location_.y_,robotinfo_->worldvtrans_.x_,robotinfo_->worldvtrans_.y_,robotinfo_->angle_.radian_);


    obstacles_info_.header.stamp= robot_info_.header.stamp;
    obstacles_info_.header.seq++;
    obstacles_info_.pos.clear();
    for(obs_info & pt : obstacles_->obs_measure_)
    {
       nubot_common::Point2d point;
       point.x=pt.world_pt.x_;
       point.y=pt.world_pt.y_;
       obstacles_info_.pos.push_back(point);
       ROS_INFO("obs_x: %f obs_y: %f",point.x, point.y);
    }

    ball_info_.header.stamp =  robot_info_.header.stamp;
    ball_info_.header.seq++;
    ball_info_.pos.x =  ball_finder_->get_ball_global_loc().x_;
    ball_info_.pos.y =  ball_finder_->get_ball_global_loc().y_;
    ball_info_.velocity.x      = ball_finder_->get_ball_velocity().x_;
    ball_info_.velocity.y      = ball_finder_->get_ball_velocity().y_;
    ball_info_.real_pos.angle  = ball_finder_->get_ball_real_loc().angle_.radian_;
    ball_info_.real_pos.radius = ball_finder_->get_ball_real_loc().radius_;
    ball_info_.velocity_known  = ball_finder_->is_velocity_known() ;
    ROS_INFO("ball_x: %f ball_y: %f",ball_info_.pos.x, ball_info_.pos.y);

    omin_vision_info_.header.stamp = robot_info_.header.stamp;
    omin_vision_info_.header.seq++;
    omin_vision_info_.ballinfo=ball_info_;
    omin_vision_info_.obstacleinfo=obstacles_info_;
    omin_vision_info_.robotinfo=robot_info_;
    omin_vision_pub_.publish(omin_vision_info_);
}

void
process()
{
    if(!colorsegment_->Segment(imginfo_->yuv_image_))
        return;

    /*get the colors of the scan_points  */
    scanpts_->process();
    if(is_show_scan_points)
        scanpts_->showScanPoints();

    /*detect the white points */
    whites_->process();
    if(is_show_whites_)
        whites_->showWhitePoints();
    if(whites_->img_white_.size()<=0)
    {
       ROS_WARN("don't detect the white points");
       return ;
    }

    /*localization */
    if(robotinfo_->isglobal_)
       robotinfo_->isglobal_=glocation_->process(whites_->weights_,whites_->robot_white_,robotinfo_->location_,robotinfo_->angle_);
    else
    {
        DPoint delta_loc=odometry_->get_world_locaton();
        Angle  delta_ang=odometry_->get_delta_angle();
        odometry_->clear(robotinfo_->angle_);
        robotinfo_->realvtrans_       = odometry_->get_real_velocity();
        robotinfo_->worldvtrans_      = odometry_->get_world_velocity();
        robotinfo_->angular_velocity_ = odometry_->get_angular_velocity();
        location_->process(whites_->weights_,whites_->robot_white_,robotinfo_->location_,robotinfo_->angle_,delta_loc,delta_ang);
    }

    /*detect the obstacles */
    obstacles_->process(colorsegment_->segment_result_,robotinfo_->location_,robotinfo_->angle_);
    if(is_show_obstacles_)
        obstacles_->show_obstacles();

    /*detect the ball */
    ball_info_.pos_known=ball_finder_->Process(colorsegment_->segment_result_,robotinfo_->location_,robotinfo_->angle_);
    if(is_show_ball_)
     {
         cv::imshow("ball",colorsegment_->segment_result_*75);
         cv::waitKey(5.0);
     }
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
    publish();
}

void
odometryupdate(const nubot_common::VelCmd & _motorinfo_msg)
{
    static std::vector<double> motor_data(nubot::MOTOR_NUMS_CONST,0);
     static ros::Time time_before=ros::Time::now();
     ros::Time time_update = _motorinfo_msg.header.stamp;
     //for(int i=0; i < MOTOR_NUMS_CONST; i++)
     //    motor_data[i]=(double)_motorinfo_msg.motordata[i];
     motor_data[0]=(double)_motorinfo_msg.Vx;
     motor_data[1]=(double)_motorinfo_msg.Vy;
     motor_data[2]=(double)_motorinfo_msg.w;

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


}// end of namespace nubot


int main(int argc, char **argv)
{ 
     ros::init(argc,argv,"omni_vision");
     ros::Time::init();
     ROS_INFO("start omni_vision process");
     nubot::Omni_Vision vision_process(argc, argv);
	 ros::spin();
	 return 0;
}
