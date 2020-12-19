#include "orca_vision/util.hpp"

#include <iostream>
//#include <iomanip>

#include "opencv2/calib3d.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace orca_vision
{

//=========================
// cv:: <==> tf2::
//=========================

void cv_to_tf2(const cv::Mat &m, tf2::Vector3 &t)
{
  assert(m.dims == 2 && m.rows == 1 && m.cols == 3 && m.type() == CV_32F);

  t = tf2::Vector3(m.at<float>(0, 0), m.at<float>(0, 1), m.at<float>(0, 2));
}

void cv_to_tf2(const cv::Mat &m, tf2::Matrix3x3 &r)
{
  assert(m.dims == 2 && m.rows == 3 && m.cols == 3 && m.type() == CV_32F);

  r = tf2::Matrix3x3(
    m.at<float>(0, 0), m.at<float>(0, 1), m.at<float>(0, 2),
    m.at<float>(1, 0), m.at<float>(1, 1), m.at<float>(1, 2),
    m.at<float>(2, 0), m.at<float>(2, 1), m.at<float>(2, 2));
}

void tf2_to_cv(const tf2::Vector3 &t, cv::Mat &m)
{
  m.create(1, 3, CV_32F);
  m.at<float>(0, 0) = static_cast<float>(t.x());
  m.at<float>(0, 1) = static_cast<float>(t.y());
  m.at<float>(0, 2) = static_cast<float>(t.z());
}

void tf2_to_cv(const tf2::Matrix3x3 &r, cv::Mat &m)
{
  m.create(3, 3, CV_32F);

  for (int i = 0; i < 3; ++i) {
    tf2::Vector3 t = r.getRow(i);
    m.at<float>(i, 0) = static_cast<float>(t.x());
    m.at<float>(i, 1) = static_cast<float>(t.y());
    m.at<float>(i, 2) = static_cast<float>(t.z());
  }
}

cv::Point3f point3f(const tf2::Vector3 &p)
{
  return {static_cast<float>(p.x()), static_cast<float>(p.y()), static_cast<float>(p.z())};
}

tf2::Vector3 vector3(const cv::Point3f &p)
{
  return {p.x, p.y, p.z};
}

void test_conversions()
{
  float data[3][3] = {{3, 3, 1}, {5, 3, 1}, {5, 5, 1}};

  cv::Mat A(3, 3, CV_32F, data);
  tf2::Matrix3x3 B;
  cv_to_tf2(A, B);
  cv::Mat C;
  tf2_to_cv(B, C);

  if (cv::sum(cv::abs(C - A)).val[0] == 0.0) {
    std::cout << "Matrix3x3 OK" << std::endl;
  } else {
    std::cout << "Matrix3x3 error" << std::endl;
  }

  cv::Mat X(1, 3, CV_32F, data);
  tf2::Vector3 Y;
  cv_to_tf2(X, Y);
  cv::Mat Z;
  tf2_to_cv(Y, Z);

  if (cv::sum(cv::abs(Z - X)).val[0] == 0.0) {
    std::cout << "Vector3 OK" << std::endl;
  } else {
    std::cout << "Vector3 error" << std::endl;
  }
}

//=========================
// Axis-angle calculations
//=========================

// TODO should return -pi to pi; run some tests

// Compute the rotation angle
float rotation_angle(const cv::Mat &r)
{
  assert(r.dims == 2 && r.rows == 3 && r.cols == 3 && r.type() == CV_32F);

  // Convert to Rodrigues format
  cv::Mat rodrigues;
  cv::Rodrigues(r, rodrigues);

  return rodrigues.at<float>(0) * rodrigues.at<float>(0)
    + rodrigues.at<float>(1) * rodrigues.at<float>(1)
    + rodrigues.at<float>(2) * rodrigues.at<float>(2);
}

// Compute the rotation angle
float rotation_angle(const tf2::Matrix3x3 &r)
{
  cv::Mat m;
  tf2_to_cv(r, m);
  return rotation_angle(m);
}

//=========================
// Rigid transforms
//=========================

// Use SVD to compute a rigid transform (R, t) such that B = R * A + t
// A and B must be Nx3
// R will be 3x3
// t will be 1x3
void rigid_transform(const cv::Mat &A, const cv::Mat &B, cv::Mat &R, cv::Mat &t)
{
  assert(A.dims == 2 && B.dims == 2 && A.cols == 3 && B.cols == 3 && A.rows == B.rows
    && A.type() == CV_32F && B.type() == CV_32F);

  // Centroid = center of cloud of points
  cv::Mat centroid_A, centroid_B; // Result is 1x3
  cv::reduce(A, centroid_A, 0, cv::REDUCE_AVG); // Average each column
  cv::reduce(B, centroid_B, 0, cv::REDUCE_AVG);

  // Translate points to the origin
  cv::Mat shift_A = cv::repeat(-centroid_A, A.rows, 1) + A;
  cv::Mat shift_B = cv::repeat(-centroid_B, B.rows, 1) + B;

  // H = shift_A.T * shift_B
  cv::Mat H = shift_A.t() * shift_B; // Result is 3x3

  // Use SVD to decompose H into W, U, Vt
  cv::Mat W, U, Vt;
  cv::SVD::compute(H, W, U, Vt); // U and Vt are 3x3

  // R = Vt.T * U.T
  R = Vt.t() * U.t();

  // Handle the reflection case
  if (cv::determinant(R) < 0) {
    Vt.col(2) *= -1;
    R = Vt.t() * U.t();
  }

  // t = (-R * centroid_A.T + centroid_B.T).T
  t = (-R * centroid_A.t() + centroid_B.t()).t();
}

void rigid_transform(std::vector<cv::Point3f> &A, std::vector<cv::Point3f> &B,
                     tf2::Transform &transform)
{
  // STL to CV
  cv::Mat mA(static_cast<int>(A.size()), 3, CV_32F, A.data());
  cv::Mat mB(static_cast<int>(B.size()), 3, CV_32F, B.data());

  cv::Mat mR;
  cv::Mat mt;
  rigid_transform(mA, mB, mR, mt);

  // CV to TF2
  // std::cout << "mR: " << mR << std::endl;
  tf2::Matrix3x3 R;
  tf2::Vector3 t;
  cv_to_tf2(mR, R);
  cv_to_tf2(mt, t);
  transform = tf2::Transform(R, t);
}

void test_rigid_transform()
{
  float A_data[4][3] = {{3, 3, 1}, {5, 3, 1}, {5, 5, 1}, {3, 5, 1}};
  float B_data[4][3] = {{-2, 3, 1}, {-2, 5, 1}, {-4, 5, 1}, {-4, 3, 1}};

  float expected_R_data[3][3] = {{0, -1, 0}, {1, 0, 0}, {0, 0, 1}};
  float expected_t_data[3] = {1, 0, 0};

  cv::Mat A(4, 3, CV_32F, A_data);
  cv::Mat B(4, 3, CV_32F, B_data);

  cv::Mat R;
  cv::Mat t;
  rigid_transform(A, B, R, t);

  cv::Mat expected_R(3, 3, CV_32F, expected_R_data);
  cv::Mat expected_t(1, 3, CV_32F, expected_t_data);

  constexpr const double max_error = 0.000001;
  double max;

  cv::minMaxIdx(cv::abs(R - expected_R), nullptr, &max, nullptr, nullptr);
  if (max < max_error) {
    std::cout << "R is OK" << std::endl;
  } else {
    std::cout << "R is wrong" << std::endl;
  }

  cv::minMaxIdx(cv::abs(t - expected_t), nullptr, &max, nullptr, nullptr);
  if (max < max_error) {
    std::cout << "T is OK" << std::endl;
  } else {
    std::cout << "T is wrong" << std::endl;
  }
}

} // namespace orca_vision
