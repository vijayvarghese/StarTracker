#include <opencv2/core/core.hpp>
#include "startracker/math/geometry.hpp"


namespace ST::math {

cv::Vec3d pixel_to_body_ray(
    double u, double v,
    double fx, double fy,
    double cx, double cy)
{
    cv::Vec3d r;
    r[0] = (u - cx) / fx;
    r[1] = (v - cy) / fy;
    r[2] = 1.0;
    
    return cv::normalize(r);
}

double AngularSeparationDeg(const cv::Vec3d& a, const cv::Vec3d& b)
{
    double dot = a.dot(b);

    dot = std::clamp(dot, -1.0, 1.0);
    return std::acos(dot) * 180.0 / CV_PI;
}


} //namespace ST::math



