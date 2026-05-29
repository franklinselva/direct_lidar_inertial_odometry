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

#include "dlio/map.hpp"

#include <pcl/console/print.h>
#include <pcl/io/pcd_io.h>

#include <iostream>

namespace dlio {

Map::Map(double leaf_size)
  : leaf_size_(leaf_size), map_(std::make_shared<Cloud>()) {
  pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
}

void Map::addKeyframe(Cloud::ConstPtr kf_cloud) {
  if (!kf_cloud || kf_cloud->empty()) return;

  Cloud::Ptr filtered = std::make_shared<Cloud>(*kf_cloud);

  this->voxelgrid_.setLeafSize(this->leaf_size_, this->leaf_size_, this->leaf_size_);
  this->voxelgrid_.setInputCloud(filtered);
  this->voxelgrid_.filter(*filtered);

  std::lock_guard<std::mutex> lock(this->mtx_);
  *this->map_ += *filtered;
}

Cloud::ConstPtr Map::cloud() const {
  std::lock_guard<std::mutex> lock(this->mtx_);
  return std::make_shared<Cloud>(*this->map_);
}

std::size_t Map::size() const {
  std::lock_guard<std::mutex> lock(this->mtx_);
  return this->map_->size();
}

bool Map::save(const std::string& path, float leaf_size) const {
  Cloud::Ptr m;
  {
    std::lock_guard<std::mutex> lock(this->mtx_);
    m = std::make_shared<Cloud>(*this->map_);
  }

  if (m->empty()) {
    std::cerr << "dlio: map is empty, nothing to save" << std::endl;
    return false;
  }

  if (leaf_size > 0.f) {
    pcl::VoxelGrid<PointType> vg;
    vg.setLeafSize(leaf_size, leaf_size, leaf_size);
    vg.setInputCloud(m);
    vg.filter(*m);
  }

  int ret = pcl::io::savePCDFileBinary(path, *m);
  return ret == 0;
}

} // namespace dlio
