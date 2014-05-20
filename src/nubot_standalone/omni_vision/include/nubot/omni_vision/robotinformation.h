#ifndef __NUBOT_VISION_ROBOTINFO_H_
#define __NUBOT_VISION_ROBOTINFO_H_


#include <opencv2/opencv.hpp>
#include "nubot/core/core.hpp"
 


namespace nubot
{ 

using std::string;


class RobotInformation
{

public:
    RobotInformation(void);
    DPoint2d realvtrans_;
    DPoint2d worldvtrans_;
	DPoint   location_;
	Angle    angle_;
    Angle    angular_velocity_;
	int      ID_;  
	bool     isglobal_;       
};



}
#endif  //!__NUBOT_VISION_ROBOTINFO_H_
