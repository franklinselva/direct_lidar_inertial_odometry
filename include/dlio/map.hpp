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

#include <pcl/filters/voxel_grid.h>

#include <mutex>
#include <string>

namespace dlio {

class Map {
public:
  explicit Map(double leaf_size = 0.25);

  // Accumulate a (world-frame) keyframe cloud into the global map.
  void addKeyframe(Cloud::ConstPtr kf_cloud);
  void addKeyframe(const Keyframe& kf) { this->addKeyframe(kf.cloud); }

  Cloud::ConstPtr cloud() const;
  std::size_t size() const;

  // Voxelize at leaf_size and write to <path> as binary PCD.
  bool save(const std::string& path, float leaf_size) const;

private:
  double leaf_size_;
  Cloud::Ptr map_;
  pcl::VoxelGrid<PointType> voxelgrid_;
  mutable std::mutex mtx_;
};

} // namespace dlio
