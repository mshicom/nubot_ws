#ifndef COMMON_H
#define COMMON_H


#endif // COMMON_H

#include "nubot/core/core.hpp"
#include <cmath>

using namespace std;

namespace nubot{

inline float
angularnorm(float _x)
{
if (_x > SINGLEPI_CONSTANT)
    _x -= DOUBLEPI_CONSTANT;
if (_x < -SINGLEPI_CONSTANT)
    _x += DOUBLEPI_CONSTANT;
return _x;
}



inline DPoint
pglobal2rel(DPoint _gpos, float _ori, DPoint _tar_gpos)
{

  DPoint pos = _gpos -  _tar_gpos;

  return DPoint(pos.x_*cos(_ori)+pos.y_*sin(_ori),
                  -pos.x_*sin(_ori)+pos.y_*cos(_ori));

}

inline DPoint
prel2global(DPoint _gpos, float _ori, DPoint _rel_pos)
{

    return  DPoint(_gpos.x_+_rel_pos.x_*cos(_ori)-_rel_pos.y_*sin(_ori),
                                        _gpos.y_+_gpos.x_*sin(_ori)+_gpos.y_*cos(_ori));

}



inline float
thetaof2p(DPoint begin, DPoint end)
{
   return  atan2(end.y_ - begin.y_, end.x_ - begin.x_);
}

inline float
thetaof2p2(DPoint begin, DPoint end)
{
    return atan2(end.x_ -  begin.x_, end.y_ - begin.y_);
}

inline float
thetaofpo(DPoint _pos)
{
   return atan2(_pos.y_,_pos.x_);
}


inline DPoint
vglobal2rel(DPoint _v, float _ori)
{

    return  DPoint(_v.x_*cos(_ori)+_v.y_*sin(_ori),
                                        -_v.x_*sin(_ori)+_v.y_*cos(_ori));

}


inline DPoint
vrel2global(DPoint _v, float _ori)
{
    return  DPoint(_v.x_*cos(_ori)-_v.y_*sin(_ori),
                                        _v.x_*sin(_ori)+_v.y_*cos(_ori));
}

inline float
sign(float _x)
{
    if(_x >= 0.00)
        return  1.0;
    else
        return -1.0;

}

}


