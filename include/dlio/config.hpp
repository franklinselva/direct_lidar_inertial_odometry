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

#include "dlio/types.hpp"

#include <limits>
#include <string>

namespace dlio {

struct Config {
  std::string version = "0.0.0";
  bool verbose = true;
  bool adaptive = true;

  SensorType sensor = SensorType::UNKNOWN; // UNKNOWN disables deskew

  struct {
    bool deskew = true;
    bool voxelize = true;
  } pointcloud;

  struct {
    double gravity = 9.80665;
    bool computeTimeOffset = false;

    struct {
      double cropBoxSize = 1.0;
      double voxelRes = 0.05;
    } preprocessing;

    struct {
      double threshD = 0.1;
      double threshR = 1.0;
    } keyframe;

    struct {
      int knn = 10;
      int kcv = 10;
      int kcc = 10;
    } submap;

    struct {
      bool approximateGravity = true;
      bool calibrateAccel = true;
      bool calibrateGyro = true;
      double calibrationTime = 3.0;
      int bufferSize = 2000;
    } imu;

    struct {
      int minNumPoints = 100;
      int kCorrespondences = 20;
      double maxCorrespondenceDistance = std::sqrt(std::numeric_limits<double>::max());
      int maxIterations = 64;
      double transformationEpsilon = 0.0005;
      double rotationEpsilon = 0.0005;
      double initLambdaFactor = 1e-9;
    } gicp;

    struct {
      double Kp = 1.0;
      double Kv = 1.0;
      double Kq = 1.0;
      double Kab = 1.0;
      double Kgb = 1.0;
      double abias_max = 1.0;
      double gbias_max = 1.0;
    } geo;
  } odom;

  struct {
    bool waitUntilMove = false;
    bool denseFiltered = true;
    double sparseLeafSize = 0.5;
  } map;

  bool imuCalibrate = true;
  Eigen::Vector3f accelBias = Eigen::Vector3f::Zero();
  Eigen::Vector3f gyroBias = Eigen::Vector3f::Zero();
  Eigen::Matrix3f accelScaleMisalign = Eigen::Matrix3f::Identity();

  Extrinsics extrinsics;
};

Config load_config(const std::string& path);

} // namespace dlio
