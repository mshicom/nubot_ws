#ifndef __NUBOT_VISION_FIELDINFOMATION_H_
#define __NUBOT_VISION_FIELDINFOMATION_H_

#include <opencv2/opencv.hpp>
#include "nubot/core/core.hpp"

namespace nubot
{ 


using std::vector;
using std::string;
class FieldInformation
{
public:

    FieldInformation();
    FieldInformation(string infopath);
	bool isInInterRect(DPoint world_pt);
	bool isInOuterRect(DPoint world_pt);
	bool isInFieldRect(DPoint world_pt,double shrink);

	bool isOppfield(DPoint world_pt);
	bool isOurfield(DPoint world_pt);


    int xline_[7];
	int yline_[5];
	DPoint opp_goal_[2];
	DPoint our_goal_[2];
	Circle centercircle_;
	Circle postcercle_[4];
    
};



}
#endif  //!__NUBOT_VISION_FIELDINFOMATION_H_

