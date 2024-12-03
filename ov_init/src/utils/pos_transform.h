#ifndef OV_INIT_POS_TRANSFORM_H
#define OV_INIT_POS_TRANSFORM_H

#include <vector>
#include <Eigen/StdVector>

//return (x,y,z) 3d vector
std::vector<double> lla2ecef(double lat,double lon,double alt);

//return (lat,lon,alt) 3d vector
std::vector<double> ecef2lla(double x,double y,double z);

//transform lla(lat,lon,alt) position to enu position,ref_lla is reference lla position
//return (x,y,z) 3d vector
std::vector<double> lla2enu(std::vector<double> lla,std::vector<double> ref_lla);

Eigen::Vector3d linear_interpolation_enu(std::pair<double,Eigen::Vector3d> pos0,
                                        std::pair<double,Eigen::Vector3d> pos1,double timestamp);
#endif
