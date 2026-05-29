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

// Live viewer: replays a PCD/IMU sequence through the push API and streams the
// estimated pose path + current pose frame + current scan into a PCL window.
// Rendering happens on the main thread (PCLVisualizer is not thread-safe); the
// onDeskewedCloud callback (main thread) stashes the latest scan, onKeyframe
// (worker thread) only appends to the thread-safe Map.

#include "dlio/config.hpp"
#include "dlio/io.hpp"
#include "dlio/lio.hpp"
#include "dlio/map.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace fs = std::filesystem;
using namespace dlio;

namespace {

std::vector<ImuSample> loadImu(const std::string& path) {
  std::vector<ImuSample> out;
  std::ifstream f(path);
  std::string line;
  while (std::getline(f, line)) {
    if (line.empty() || line[0] == '#') continue;
    std::replace(line.begin(), line.end(), ',', ' ');
    std::istringstream ss(line);
    ImuSample s;
    if (ss >> s.stamp >> s.ang_vel[0] >> s.ang_vel[1] >> s.ang_vel[2]
           >> s.lin_accel[0] >> s.lin_accel[1] >> s.lin_accel[2])
      out.push_back(s);
  }
  std::sort(out.begin(), out.end(), [](const ImuSample& a, const ImuSample& b){ return a.stamp < b.stamp; });
  return out;
}

struct ScanFile { double stamp; std::string path; };

std::vector<ScanFile> listScans(const std::string& dir) {
  std::vector<ScanFile> out;
  for (const auto& e : fs::directory_iterator(dir)) {
    if (e.path().extension() != ".pcd") continue;
    try { out.push_back({std::stod(e.path().stem().string()), e.path().string()}); }
    catch (...) {}
  }
  std::sort(out.begin(), out.end(), [](const ScanFile& a, const ScanFile& b){ return a.stamp < b.stamp; });
  return out;
}

Eigen::Affine3f poseToAffine(const State& s) {
  Eigen::Affine3f a = Eigen::Affine3f::Identity();
  a.translate(s.p);
  a.rotate(s.q);
  return a;
}

} // namespace

int main(int argc, char** argv) {
  if (argc < 5) {
    std::cerr << "usage: " << argv[0]
              << " <config.yaml> <pcd_dir> <imu.csv> <out_dir> [spin_ms]" << std::endl;
    return 1;
  }
  const std::string config_path = argv[1], pcd_dir = argv[2], imu_csv = argv[3], out_dir = argv[4];
  const int spin_ms = (argc > 5) ? std::stoi(argv[5]) : 1;

  Config cfg = load_config(config_path);
  LIO lio(cfg);
  Map map(cfg.map.sparseLeafSize);

  lio.setExtrinsics(cfg.extrinsics);
  lio.setImuIntrinsics({cfg.accelBias, cfg.gyroBias, cfg.accelScaleMisalign});
  lio.setImuCalibration(cfg.imuCalibrate);

  // Latest deskewed scan, stashed on the main thread.
  Cloud::ConstPtr latest_scan;
  lio.onDeskewedCloud = [&](Cloud::ConstPtr c, const State&, double){ latest_scan = c; };
  lio.onKeyframe = [&](const Keyframe& kf){ map.addKeyframe(kf.cloud); };

  // Viewer
  pcl::visualization::PCLVisualizer::Ptr viewer(
      new pcl::visualization::PCLVisualizer("DLIO live"));
  viewer->setBackgroundColor(0.05, 0.05, 0.08);
  viewer->addCoordinateSystem(1.0);                  // world origin
  viewer->addCoordinateSystem(1.5, Eigen::Affine3f::Identity(), "pose"); // current pose
  viewer->initCameraParameters();
  viewer->setCameraPosition(-10, -10, 8, 0, 0, 0, 0, 0, 1);

  auto path = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();   // pose path

  std::vector<ImuSample> imu = loadImu(imu_csv);
  std::vector<ScanFile> scans = listScans(pcd_dir);
  std::cout << "loaded " << imu.size() << " IMU, " << scans.size() << " scans" << std::endl;

  const double imu_lookahead = 0.1;
  std::size_t ii = 0;
  for (const auto& sf : scans) {
    if (viewer->wasStopped()) break;

    while (ii < imu.size() && imu[ii].stamp <= sf.stamp + imu_lookahead)
      lio.addImu(imu[ii++]);

    auto cloud = pcl::make_shared<Cloud>();
    if (pcl::io::loadPCDFile(sf.path, *cloud) != 0) continue;
    if (!lio.addScan(cloud, sf.stamp)) { viewer->spinOnce(spin_ms); continue; }

    State s = lio.getState();

    // pose path
    path->push_back(pcl::PointXYZ(s.p.x(), s.p.y(), s.p.z()));
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> green(path, 40, 230, 90);
    if (!viewer->updatePointCloud(path, green, "path"))
      viewer->addPointCloud(path, green, "path");
    viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "path");

    // current pose frame
    viewer->updateCoordinateSystemPose("pose", poseToAffine(s));

    // current scan (world frame), white, small
    if (latest_scan && !latest_scan->empty()) {
      pcl::visualization::PointCloudColorHandlerCustom<PointType> white(
          std::const_pointer_cast<Cloud>(latest_scan), 200, 200, 210);
      if (!viewer->updatePointCloud<PointType>(latest_scan, white, "scan"))
        viewer->addPointCloud<PointType>(latest_scan, white, "scan");
      viewer->setPointCloudRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "scan");
    }

    viewer->spinOnce(spin_ms);
  }

  std::cout << "replay done — " << lio.getTrajectory().size() << " poses, "
            << lio.getKeyframes().size() << " keyframes. Close window to save & exit." << std::endl;

  // Show the accumulated map after replay
  if (Cloud::ConstPtr m = map.cloud(); m && !m->empty()) {
    pcl::visualization::PointCloudColorHandlerGenericField<PointType> field(
        std::const_pointer_cast<Cloud>(m), "intensity");
    viewer->addPointCloud<PointType>(m, field, "map");
  }

  while (!viewer->wasStopped()) viewer->spinOnce(50);

  fs::create_directories(out_dir);
  map.save(out_dir + "/dlio_map.pcd", cfg.map.sparseLeafSize);
  io::write_tum_trajectory(out_dir + "/trajectory.txt", lio.getTrajectory(), lio.getTrajectoryStamps());
  io::write_keyframes(out_dir + "/keyframes", lio.getKeyframes());
  return 0;
}
