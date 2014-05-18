#include "nubot/omni_vision/robotinformation.h"

using namespace nubot;

RobotInformation::RobotInformation(void):location_(0,0),angle_(0),angular_velocity_(0),
ID_(1),isglobal_(true),realvtrans_(0,0),worldvtrans_(0,0)
{
}
