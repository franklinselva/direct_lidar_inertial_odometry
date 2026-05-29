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

#include "dlio/io.hpp"

#include <pcl/io/pcd_io.h>

#include <filesystem>
#include <fstream>
#include <iostream>

namespace dlio::io {

namespace {
void write_tum_row(std::ostream& os, double stamp, const Pose& p) {
  os << std::fixed << std::setprecision(9) << stamp << " "
     << std::setprecision(6)
     << p.p.x() << " " << p.p.y() << " " << p.p.z() << " "
     << p.q.x() << " " << p.q.y() << " " << p.q.z() << " " << p.q.w() << "\n";
}
}

bool write_tum_trajectory(const std::string& path,
                          const std::vector<Pose>& poses,
                          const std::vector<double>& stamps) {
  if (poses.size() != stamps.size()) {
    std::cerr << "dlio: trajectory pose/stamp count mismatch" << std::endl;
    return false;
  }

  std::ofstream f(path);
  if (!f) {
    std::cerr << "dlio: could not open '" << path << "' for writing" << std::endl;
    return false;
  }

  for (std::size_t i = 0; i < poses.size(); ++i) {
    write_tum_row(f, stamps[i], poses[i]);
  }
  return true;
}

bool write_keyframes(const std::string& dir, const std::vector<Keyframe>& keyframes) {
  std::error_code ec;
  std::filesystem::create_directories(dir, ec);
  if (ec) {
    std::cerr << "dlio: could not create directory '" << dir << "': " << ec.message() << std::endl;
    return false;
  }

  std::ofstream poses(dir + "/keyframe_poses.txt");
  if (!poses) {
    std::cerr << "dlio: could not open keyframe_poses.txt for writing" << std::endl;
    return false;
  }

  bool ok = true;
  for (std::size_t i = 0; i < keyframes.size(); ++i) {
    write_tum_row(poses, keyframes[i].stamp, keyframes[i].pose);
    if (keyframes[i].cloud && !keyframes[i].cloud->empty()) {
      std::string p = dir + "/keyframe_" + std::to_string(i) + ".pcd";
      if (pcl::io::savePCDFileBinary(p, *keyframes[i].cloud) != 0) {
        std::cerr << "dlio: failed to save " << p << std::endl;
        ok = false;
      }
    }
  }
  return ok;
}

} // namespace dlio::io
