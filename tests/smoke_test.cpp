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

#include "dlio/config.hpp"
#include "dlio/lio.hpp"
#include "dlio/map.hpp"

#include <cmath>
#include <iostream>

using namespace dlio;

static Cloud::Ptr makeCorner(float dx, float dy) {
  // three orthogonal planes forming a corner, shifted by (dx, dy)
  auto cloud = std::make_shared<Cloud>();
  for (int i = 0; i < 60; ++i) {
    for (int j = 0; j < 60; ++j) {
      float a = 2.f + i * 0.1f;
      float b = 2.f + j * 0.1f;
      PointType p1; p1.x = a + dx; p1.y = b + dy; p1.z = 0.f; cloud->push_back(p1); // z=0 plane
      PointType p2; p2.x = a + dx; p2.y = 0.f;    p2.z = b;   cloud->push_back(p2); // y=0 plane
      PointType p3; p3.x = 0.f;    p3.y = a + dy; p3.z = b;   cloud->push_back(p3); // x=0 plane
    }
  }
  cloud->width = cloud->size();
  cloud->height = 1;
  cloud->is_dense = true;
  return cloud;
}

static bool finite(const State& s) {
  return std::isfinite(s.p.x()) && std::isfinite(s.p.y()) && std::isfinite(s.p.z()) &&
         std::isfinite(s.q.w());
}

int main() {
  Config cfg;            // library defaults
  cfg.verbose = false;
  cfg.sensor = SensorType::UNKNOWN;   // disable deskew (synthetic clouds, no point times)
  cfg.pointcloud.deskew = false;
  cfg.odom.imu.calibrationTime = 0.5; // shorten calibration for the test
  cfg.odom.gicp.minNumPoints = 64;

  LIO lio(cfg);
  Map map(0.25);

  int kf_count = 0;
  lio.onKeyframe = [&](const Keyframe& kf) { map.addKeyframe(kf.cloud); ++kf_count; };

  // Feed IMU (200 Hz) over [0, 6] s; gravity on +z so it stays still.
  const double dt = 1.0 / 200.0;
  for (double t = 0.0; t <= 6.0 + 1e-9; t += dt) {
    ImuSample imu;
    imu.stamp = t;
    imu.ang_vel = Eigen::Vector3f::Zero();
    imu.lin_accel = Eigen::Vector3f(0.f, 0.f, 9.80665f);
    lio.addImu(imu);
  }

  // Feed a sequence of scans (all IMU already buffered up to t=6).
  int processed = 0;
  for (int k = 0; k < 8; ++k) {
    double stamp = 3.0 + 0.1 * k;
    float shift = 0.02f * k;
    if (lio.addScan(makeCorner(shift, shift), stamp)) {
      ++processed;
    }
    State s = lio.getState();
    if (!finite(s)) {
      std::cerr << "FAIL: non-finite state at scan " << k << std::endl;
      return 1;
    }
  }

  std::cout << "processed scans:     " << processed << std::endl;
  std::cout << "trajectory length:   " << lio.getTrajectory().size() << std::endl;
  std::cout << "keyframes:           " << lio.getKeyframes().size() << std::endl;
  std::cout << "map points:          " << map.size() << std::endl;

  if (!lio.initialized()) {
    std::cerr << "FAIL: LIO did not initialize" << std::endl;
    return 1;
  }
  if (processed < 1) {
    std::cerr << "FAIL: no scans processed" << std::endl;
    return 1;
  }
  if (lio.getKeyframes().empty()) {
    std::cerr << "FAIL: no keyframes created" << std::endl;
    return 1;
  }

  State final = lio.getState();
  std::cout << "final position: " << final.p.transpose() << std::endl;

  std::cout << "PASS" << std::endl;
  return 0;
}
