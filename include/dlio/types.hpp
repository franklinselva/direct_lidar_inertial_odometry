/***********************************************************
 *                                                         *
 * Copyright (c)                                           *
 *                                                         *
 * The Verifiable & Control-Theoretic Robotics (VECTR) Lab *
 * University of California, Los Angeles                   *
 *                                                         *
 * Authors: Kenny J. Chen, Ryan Nemiroff, Brett T. Lopez   *
 * Contact: {kennyjchen, ryguyn, btlopez}@ucla.edu         *
 *                                                         *
 ***********************************************************/

#pragma once

#include "dlio/point.hpp"

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <iomanip>
#include <sstream>
#include <string>

namespace dlio {

using PointType = dlio::Point;
using Cloud = pcl::PointCloud<PointType>;

enum class SensorType { OUSTER, VELODYNE, HESAI, LIVOX, UNKNOWN };

template <typename T>
inline std::string to_string_with_precision(const T a_value, const int n = 6) {
  std::ostringstream out;
  out.precision(n);
  out << std::fixed << a_value;
  return out.str();
}

struct ImuSample {
  double stamp = 0.0;
  Eigen::Vector3f ang_vel = Eigen::Vector3f::Zero();
  Eigen::Vector3f lin_accel = Eigen::Vector3f::Zero();
};

struct Pose {
  Eigen::Vector3f p = Eigen::Vector3f::Zero();
  Eigen::Quaternionf q = Eigen::Quaternionf::Identity();
};

struct ImuBias {
  Eigen::Vector3f gyro = Eigen::Vector3f::Zero();
  Eigen::Vector3f accel = Eigen::Vector3f::Zero();
};

struct Frames {
  Eigen::Vector3f b = Eigen::Vector3f::Zero();
  Eigen::Vector3f w = Eigen::Vector3f::Zero();
};

struct Velocity {
  Frames lin;
  Frames ang;
};

struct State {
  Eigen::Vector3f p = Eigen::Vector3f::Zero();   // position in world frame
  Eigen::Quaternionf q = Eigen::Quaternionf::Identity(); // orientation in world frame
  Velocity v;
  ImuBias b;  // imu biases in body frame
};

struct Keyframe {
  Pose pose;
  Cloud::ConstPtr cloud;
  double stamp = 0.0;
};

struct ImuIntrinsics {
  Eigen::Vector3f accelBias = Eigen::Vector3f::Zero();
  Eigen::Vector3f gyroBias = Eigen::Vector3f::Zero();
  Eigen::Matrix3f accelScaleMisalign = Eigen::Matrix3f::Identity();
};

struct Extrinsics {
  struct SE3 {
    Eigen::Vector3f t = Eigen::Vector3f::Zero();
    Eigen::Matrix3f R = Eigen::Matrix3f::Identity();
  };
  SE3 baselink2imu;
  SE3 baselink2lidar;
  Eigen::Matrix4f baselink2imu_T = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f baselink2lidar_T = Eigen::Matrix4f::Identity();

  // Rebuild the 4x4 transforms from the t/R of each SE3.
  void recompute() {
    baselink2imu_T = Eigen::Matrix4f::Identity();
    baselink2imu_T.block(0, 3, 3, 1) = baselink2imu.t;
    baselink2imu_T.block(0, 0, 3, 3) = baselink2imu.R;
    baselink2lidar_T = Eigen::Matrix4f::Identity();
    baselink2lidar_T.block(0, 3, 3, 1) = baselink2lidar.t;
    baselink2lidar_T.block(0, 0, 3, 3) = baselink2lidar.R;
  }
};

} // namespace dlio
