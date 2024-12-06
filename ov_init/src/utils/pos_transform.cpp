#include "pos_transform.h"
#include <cmath>
#include <iostream>
#define RE_WGS84 6378137.0 //earth semimajor axis (WGS84) (m) 
#define FE_WGS84 (1.0/298.257223563) //earth flattening (WGS84)


std::vector<double> lla2ecef(double lat,double lon,double alt)
{
    lat = lat / 180.0 * M_PI;
    lon = lon / 180.0 * M_PI;
    double a = RE_WGS84;
    double f = FE_WGS84;
    double b = (1.0 - f) * a;
    double e = std::sqrt(1.0 - std::pow((b/a),2));
    double N = a / std::sqrt(1-std::pow(e*std::sin(lat),2));
    double x = (N + alt) * std::cos(lat) * std::cos(lon);
    double y = (N + alt) * std::cos(lat) * std::sin(lon);
    double z = (N * (1.0-e*e) + alt) * std::sin(lat);
    
    std::vector<double> pos(3);
    pos[0] = x;
    pos[1] = y;
    pos[2] = z;
    return pos;
}

std::vector<double> ecef2lla(double x,double y,double z)
{
    double a = RE_WGS84;
    double f = FE_WGS84;
    double e2 = f*(2.0-f);
    double p = std::sqrt(x*x+y*y);
    double phi = std::atan2(z,p*(1.0-e2));
    double h = 0.0;
    double lastPhi = phi + 1.0;
    while(std::fabs(phi-lastPhi) > 1e-12)
    {
        lastPhi = phi;
        double sinPhi = std::sin(phi);
        double cosPhi = std::cos(phi);
        double n = a/std::sqrt(1.0-e2*sinPhi*sinPhi);
        h = p/cosPhi - n;
        phi = std::atan2(z,p*(1.0-e2*n/(n+h)));
    }
    double lon = std::atan2(y,x);
    double lat = phi;
    
    std::vector<double> lla(3);
    lla[0] = lat/M_PI*180.0;
    lla[1] = lon/M_PI*180.0;
    lla[2] = h;
    return lla;
}

std::vector<double> lla2enu(std::vector<double> lla,std::vector<double> ref_lla)
{
    std::vector<double> pos = lla2ecef(lla[0],lla[1],lla[2]);
    std::vector<double> ref_pos = lla2ecef(ref_lla[0],ref_lla[1],ref_lla[2]);
    double sin_ref_lat = std::sin(ref_lla[0]/180*M_PI);
    double cos_ref_lat = std::cos(ref_lla[0]/180*M_PI);
    double sin_ref_lon = std::sin(ref_lla[1]/180*M_PI);
    double cos_ref_lon = std::cos(ref_lla[1]/180*M_PI);
    double dx = pos[0] - ref_pos[0];
    double dy = pos[1] - ref_pos[1];
    double dz = pos[2] - ref_pos[2];

    double e = -sin_ref_lon * dx + cos_ref_lon * dy;
    double n = -sin_ref_lat * cos_ref_lon * dx - sin_ref_lat * sin_ref_lon * dy + cos_ref_lat * dz;
    double u = cos_ref_lat * cos_ref_lon * dx + cos_ref_lat * sin_ref_lon * dy + sin_ref_lat * dz;
    
    std::vector<double> enu(3);
    enu[0] = e;
    enu[1] = n;
    enu[2] = u;
    return enu;
}

std::vector<double> enu2lla(std::vector<double> enu,std::vector<double> ref_lla)
{
    std::vector<double> ref_pos = lla2ecef(ref_lla[0],ref_lla[1],ref_lla[2]);
    double sin_ref_lat = std::sin(ref_lla[0]/180*M_PI);
    double cos_ref_lat = std::cos(ref_lla[0]/180*M_PI);
    double sin_ref_lon = std::sin(ref_lla[1]/180*M_PI);
    double cos_ref_lon = std::cos(ref_lla[1]/180*M_PI);
    double e = enu[0], n = enu[1], u = enu[2];

    double dx = -e*sin_ref_lon-n*sin_ref_lat*cos_ref_lon+u*cos_ref_lat*cos_ref_lon;
    double dy = e*cos_ref_lon-n*sin_ref_lat*sin_ref_lon+u*cos_ref_lat*sin_ref_lon;
    double dz = n*cos_ref_lat+u*sin_ref_lat;

    std::vector<double> pos(3);
    pos[0] = ref_pos[0] + dx;
    pos[1] = ref_pos[1] + dy;
    pos[2] = ref_pos[2] + dz;

    std::vector<double> lla = ecef2lla(pos[0],pos[1],pos[2]);

    return lla;
}

Eigen::Vector3d linear_interpolation_enu(std::pair<double,Eigen::Vector3d> pos0,
                    std::pair<double,Eigen::Vector3d> pos1,double timestamp)
{
    Eigen::Vector3d v = (pos1.second - pos0.second)/(pos1.first - pos0.first);
    Eigen::Vector3d pos; 
    if(fabs(pos0.first - timestamp) < fabs(pos1.first - timestamp))
    {
        pos = pos0.second + v * (timestamp - pos0.first);
    }
    else
    {
        pos = pos1.second - v * (pos1.first - timestamp);
    }
    return pos;
}