#include "nubot/omni_vision/fieldinformation.h"

using namespace nubot;

FieldInformation::FieldInformation()
{
	xline_[0]=900;
	xline_[1]=825;
	xline_[2]=675;
	xline_[3]=0;
	xline_[4]=-675;
	xline_[5]=-825;
	xline_[6]=-900;
	
	yline_[0]=600;
	yline_[1]=250;
	yline_[2]=150;
	yline_[3]=-150;
	yline_[4]=-250;
	yline_[5]=-600;

	centercircle_.radius_=200;
	centercircle_.center_=DPoint2d(0,0);
	for(size_t i=0; i< 4;i++)
	   postcercle_[i].radius_=75;
	postcercle_[0].center_=DPoint2d(xline_[0],-yline_[0]);
	postcercle_[1].center_=DPoint2d(xline_[0],yline_[0]);
	postcercle_[2].center_=DPoint2d(-xline_[0],yline_[0]);
	postcercle_[3].center_=DPoint2d(-xline_[0],-yline_[0]);

    opp_goal_[0] = DPoint(xline_[6],yline_[3]);
	opp_goal_[0] = DPoint(xline_[6],yline_[2]);
	our_goal_[0] = DPoint(xline_[0],yline_[3]);
	our_goal_[0] = DPoint(xline_[0],yline_[2]);
}

FieldInformation::FieldInformation(string infopath)
{
	
}

bool 
FieldInformation::isInInterRect(DPoint world_pt)
{
	 return (world_pt.x_>xline_[6]&&world_pt.x_<xline_[0]&&world_pt.y_>yline_[5]&&world_pt.y_<yline_[0]);
}

bool 
FieldInformation::isInOuterRect(DPoint world_pt)
{
	return (world_pt.x_>xline_[6]-100&&world_pt.x_<xline_[0]+100&&world_pt.y_>yline_[5]-50&&world_pt.y_<yline_[0]+50);
}

bool 
FieldInformation::isInFieldRect(DPoint world_pt,double shrink)
{
	return (world_pt.x_>xline_[6]-shrink&&world_pt.x_<xline_[0]+shrink&&world_pt.y_>yline_[5]-shrink&&world_pt.y_<yline_[0]+shrink);
}	

bool 
FieldInformation::isOppfield(DPoint world_pt)
{
	return (world_pt.x_>0);
}

bool 
FieldInformation::isOurfield(DPoint world_pt)
{
	return (world_pt.x_<0);
}
