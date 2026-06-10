#pragma once
#include <opencv2/core/core.hpp>

namespace ST::math {

/**
 * @brief Calculate body frame vector from centroid. Using pin hole camera
 * model.
 * @param u,v,fx,fy Camera intrinsic matrix.
 * @param cx,cy Centroid x,y.
 * @return cv::normalize(cv::Vector2d) - Normalized body frame vector.
 */
cv::Vec3d pixel_to_body_ray(double u, double v, double fx, double fy, double cx,
                            double cy);

/**
 * @brief Calculate the angular seperation of two given vectors.
 * @param a Reference to vector A (cv::Vec3d)
 * @param b Reference to vector B (cv::Vec3d)
 * @return (double) Angular seperation in degrees.
 * @todo Need improvement later (Naming, Implimentation)
 */
double AngularSeparationDeg(const cv::Vec3d &a, const cv::Vec3d &b);

} // namespace ST::math