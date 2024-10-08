#pragma once
#include <cmath>
#include <pcl/point_types.h>

using PointType = pcl::PointXYZI;

inline double rad2deg(double radians)
{
    return radians * 180.0 / M_PI;
}

inline double deg2rad(double degrees)
{
    return degrees * M_PI / 180.0;
}
