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

// File-replay driver: feeds timestamped PCD scans + an IMU CSV through the
// push API, then writes the map, trajectory and keyframes. A real sensor uses
// the exact same calls (addImu / addScan), fed from a driver thread.
//
// IMU CSV rows:  stamp,wx,wy,wz,ax,ay,az
// Scan files:    <pcd_dir>/<stamp>.pcd  (filename stem parsed as the timestamp)

#include "dlio/config.hpp"
#include "dlio/io.hpp"
#include "dlio/lio.hpp"
#include "dlio/map.hpp"

#include <pcl/io/pcd_io.h>

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
  if (!f) { std::cerr << "could not open IMU csv: " << path << std::endl; return out; }
  std::string line;
  while (std::getline(f, line)) {
    if (line.empty() || line[0] == '#') continue;
    std::replace(line.begin(), line.end(), ',', ' ');
    std::istringstream ss(line);
    ImuSample s;
    if (ss >> s.stamp >> s.ang_vel[0] >> s.ang_vel[1] >> s.ang_vel[2]
           >> s.lin_accel[0] >> s.lin_accel[1] >> s.lin_accel[2]) {
      out.push_back(s);
    }
  }
  std::sort(out.begin(), out.end(), [](const ImuSample& a, const ImuSample& b){ return a.stamp < b.stamp; });
  return out;
}

struct ScanFile { double stamp; std::string path; };

std::vector<ScanFile> listScans(const std::string& dir) {
  std::vector<ScanFile> out;
  for (const auto& e : fs::directory_iterator(dir)) {
    if (e.path().extension() != ".pcd") continue;
    try {
      out.push_back({std::stod(e.path().stem().string()), e.path().string()});
    } catch (...) {
      std::cerr << "skipping (stem not a timestamp): " << e.path() << std::endl;
    }
  }
  std::sort(out.begin(), out.end(), [](const ScanFile& a, const ScanFile& b){ return a.stamp < b.stamp; });
  return out;
}

} // namespace

int main(int argc, char** argv) {
  if (argc < 5) {
    std::cerr << "usage: " << argv[0]
              << " <config.yaml> <pcd_dir> <imu.csv> <out_dir>" << std::endl;
    return 1;
  }
  const std::string config_path = argv[1];
  const std::string pcd_dir = argv[2];
  const std::string imu_csv = argv[3];
  const std::string out_dir = argv[4];

  Config cfg = load_config(config_path);

  LIO lio(cfg);
  Map map(cfg.map.sparseLeafSize);
  lio.start();

  lio.onKeyframe = [&](const Keyframe& kf) { map.addKeyframe(kf.cloud); };

  std::vector<ImuSample> imu = loadImu(imu_csv);
  std::vector<ScanFile> scans = listScans(pcd_dir);
  std::cout << "loaded " << imu.size() << " IMU samples, " << scans.size() << " scans" << std::endl;

  // Merge-replay: push all IMU up to the next scan stamp, then the scan.
  std::size_t ii = 0;
  for (const auto& sf : scans) {
    while (ii < imu.size() && imu[ii].stamp <= sf.stamp) {
      lio.addImu(imu[ii++]);
    }
    auto cloud = std::make_shared<Cloud>();
    if (pcl::io::loadPCDFile(sf.path, *cloud) != 0) {
      std::cerr << "failed to load " << sf.path << std::endl;
      continue;
    }
    lio.addScan(cloud, sf.stamp);
  }
  // drain remaining IMU
  while (ii < imu.size()) lio.addImu(imu[ii++]);

  // Save outputs
  fs::create_directories(out_dir);
  const std::string map_pcd = out_dir + "/dlio_map.pcd";
  const std::string traj_txt = out_dir + "/trajectory.txt";
  const std::string kf_dir = out_dir + "/keyframes";

  bool ok_map = map.save(map_pcd, cfg.map.sparseLeafSize);
  bool ok_traj = io::write_tum_trajectory(traj_txt, lio.getTrajectory(), lio.getTrajectoryStamps());
  bool ok_kf = io::write_keyframes(kf_dir, lio.getKeyframes());

  std::cout << "\nmap         -> " << map_pcd  << (ok_map  ? " [ok]" : " [FAILED]") << std::endl;
  std::cout << "trajectory  -> " << traj_txt << (ok_traj ? " [ok]" : " [FAILED]") << std::endl;
  std::cout << "keyframes   -> " << kf_dir   << (ok_kf   ? " [ok]" : " [FAILED]") << std::endl;

  return (ok_map && ok_traj && ok_kf) ? 0 : 1;
}
