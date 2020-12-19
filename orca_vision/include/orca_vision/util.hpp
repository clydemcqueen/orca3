#ifndef VISION_UTIL_H
#define VISION_UTIL_H

#include "geometry_msgs/msg/pose.hpp"
#include "opencv2/core/mat.hpp"
#include "tf2/LinearMath/Transform.h"

namespace orca_vision
{

//=========================
// std::
//=========================

template<typename T>
T destructive_median(std::vector<T> &v)
{
  if (v.size() == 0) {
    return 0;
  } else {
    std::nth_element(v.begin(), v.begin() + v.size() / 2, v.end());
    return v[v.size() / 2];
  }
}

//=========================
// cv:: <==> tf2::
//=========================

// cv::Mats must be CV_32F
// cv::Mats that represent vectors must be 1x3

void cv_to_tf2(const cv::Mat &m, tf2::Vector3 &t);
void cv_to_tf2(const cv::Mat &m, tf2::Matrix3x3 &r);

void tf2_to_cv(const tf2::Vector3 &t, cv::Mat &m);
void tf2_to_cv(const tf2::Matrix3x3 &r, cv::Mat &m);

cv::Point3f point3f(const tf2::Vector3 &p);
tf2::Vector3 vector3(const cv::Point3f &p);

void test_conversions();

//=========================
// Axis-angle calculations
//=========================

float rotation_angle(const cv::Mat &r);
float rotation_angle(const tf2::Matrix3x3 &r);

//=========================
// Rigid transforms
//=========================

// Use SVD to compute a rigid transform (R, t) such that B = R * A + t
// A and B must be Nx3
// cv::Mats must be CV_32F
// R will be 3x3
// t will be 1x3
void rigid_transform(const cv::Mat &A, const cv::Mat &B, cv::Mat &R, cv::Mat &t);
void rigid_transform(std::vector<cv::Point3f> &A, std::vector<cv::Point3f> &B,
                     tf2::Transform &transform);

void test_rigid_transform();

} // namespace orca_vision

#endif // VISION_UTIL_H
