#include "nubot/world_model/world_model.h"

nubot::world_model::world_model()
{
}
nubot::world_model::~world_model()
{

}

void nubot::world_model::updateOminivision(const nubot_standalone::OminiVisionInfo & omni_info)
{
    ros::Time time_update = omni_info.header.stamp;
    robot_info_.set_time(time_update);
    robot_info_.set_robot_vec(DPoint2d(omni_info.robotinfo.vtrans.x,omni_info.robotinfo.vtrans.y));
    robot_info_.set_robot_loc(DPoint2d(omni_info.robotinfo.pos.x,omni_info.robotinfo.pos.y));
    robot_info_.set_robot_w(omni_info.robotinfo.vrot);
    robot_info_.set_robot_head(Angle(omni_info.robotinfo.heading.theta));

    ball_object omni_ball;
    omni_ball.set_ball_global_loc(DPoint(omni_info.ballinfo.pos.x,omni_info.ballinfo.pos.y));
    omni_ball.set_ball_pos_known(omni_info.ballinfo.pos_known);
    omni_ball.set_ball_vec(DPoint(omni_info.ballinfo.velocity.x,omni_info.ballinfo.velocity.y));
    omni_ball.set_ball_real_loc(PPoint(Angle(omni_info.ballinfo.real_pos.angle),omni_info.ballinfo.real_pos.radius));
    omni_ball.set_ball_time(time_update);
    ball_info_.set_omni_ball(omni_ball);

    std::vector< obstacle_object > obstacles;
    obstacles.reserve(omni_info.obstacleinfo.length );
    for(std::size_t i = 0; i< omni_info.obstacleinfo.length ; i++)
    {
        obstacle_object obs_temp;
        obs_temp.set_obstacle_loc(DPoint2d(omni_info.obstacleinfo.pos[i].x,omni_info.obstacleinfo.pos[i].y));
        obstacles.push_back(obs_temp);
    }
    obstacles_.set_omni_obstacles(obstacles);
    obstacles_.set_time(time_update);
}
void nubot::world_model::update()
{
    ball_info_.update();
    robot_info_.update();
    for(std::size_t i = 0 ; i < OUR_TEAM ;i ++ )
        teamnate_[i].update();
    obstacles_.update();
}
