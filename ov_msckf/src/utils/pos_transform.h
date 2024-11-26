#ifndef OV_MSCKF_POS_TRANSFORM_H
#define OV_MSCKF_POS_TRANSFORM_H

#include <vector>

//return (x,y,z) 3d vector
std::vector<double> lla2ecef(double lat,double lon,double alt);

//return (lat,lon,alt) 3d vector
std::vector<double> ecef2lla(double x,double y,double z);

//transform lla(lat,lon,alt) position to enu position,ref_lla is reference lla position
//return (x,y,z) 3d vector
std::vector<double> lla2enu(std::vector<double> lla,std::vector<double> ref_lla);

#endif
