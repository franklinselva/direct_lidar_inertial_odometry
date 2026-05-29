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

#include "dlio/dense_map.hpp"

#include <pcl/io/pcd_io.h>

#include <cmath>

namespace dlio {

DenseMap::DenseMap(float leaf_size, float carve_margin, float ground_z)
  : leaf_(leaf_size > 0.f ? leaf_size : 0.10f),
    carve_margin_(carve_margin > 0.f ? carve_margin : 0.f),
    ground_z_(ground_z) {}

std::int64_t DenseMap::keyIdx(std::int64_t ix, std::int64_t iy, std::int64_t iz) const {
  const std::int64_t mask = (1LL << 21) - 1;  // +/- ~1M voxels per axis
  return ((ix & mask) << 42) | ((iy & mask) << 21) | (iz & mask);
}

std::int64_t DenseMap::key(float x, float y, float z) const {
  return this->keyIdx(static_cast<std::int64_t>(std::floor(x / leaf_)),
                      static_cast<std::int64_t>(std::floor(y / leaf_)),
                      static_cast<std::int64_t>(std::floor(z / leaf_)));
}

// Amanatides–Woo voxel traversal from origin to endpoint; bump `pass` on every
// voxel strictly BEFORE the endpoint voxel (free space along the ray).
void DenseMap::carve(const Eigen::Vector3f& o, const Eigen::Vector3f& e) {
  std::int64_t ix = static_cast<std::int64_t>(std::floor(o.x() / leaf_));
  std::int64_t iy = static_cast<std::int64_t>(std::floor(o.y() / leaf_));
  std::int64_t iz = static_cast<std::int64_t>(std::floor(o.z() / leaf_));
  const std::int64_t ex = static_cast<std::int64_t>(std::floor(e.x() / leaf_));
  const std::int64_t ey = static_cast<std::int64_t>(std::floor(e.y() / leaf_));
  const std::int64_t ez = static_cast<std::int64_t>(std::floor(e.z() / leaf_));

  const Eigen::Vector3f d = e - o;
  const int sx = d.x() > 0 ? 1 : (d.x() < 0 ? -1 : 0);
  const int sy = d.y() > 0 ? 1 : (d.y() < 0 ? -1 : 0);
  const int sz = d.z() > 0 ? 1 : (d.z() < 0 ? -1 : 0);

  const float inf = std::numeric_limits<float>::infinity();
  auto tdelta = [&](float dd) { return dd != 0.f ? std::fabs(leaf_ / dd) : inf; };
  float tDeltaX = tdelta(d.x()), tDeltaY = tdelta(d.y()), tDeltaZ = tdelta(d.z());

  // distance to first voxel boundary along each axis
  auto tfirst = [&](float oc, std::int64_t i, int s, float dd) -> float {
    if (s == 0 || dd == 0.f) return inf;
    const float bound = (i + (s > 0 ? 1 : 0)) * leaf_;
    return (bound - oc) / dd;
  };
  float tMaxX = tfirst(o.x(), ix, sx, d.x());
  float tMaxY = tfirst(o.y(), iy, sy, d.y());
  float tMaxZ = tfirst(o.z(), iz, sz, d.z());

  const int max_steps = 4096;  // safety bound
  for (int step = 0; step < max_steps; ++step) {
    if (ix == ex && iy == ey && iz == ez) break;  // reached endpoint voxel
    if (tMaxX <= tMaxY && tMaxX <= tMaxZ)      { ix += sx; tMaxX += tDeltaX; }
    else if (tMaxY <= tMaxZ)                    { iy += sy; tMaxY += tDeltaY; }
    else                                        { iz += sz; tMaxZ += tDeltaZ; }
    if (ix == ex && iy == ey && iz == ez) break;  // don't carve the endpoint
    // protect the ground plane: grazing rays to far ground points would
    // otherwise carve the floor out from under themselves.
    if ((iz + 0.5f) * leaf_ < ground_z_) continue;
    ++this->grid_[this->keyIdx(ix, iy, iz)].pass;
  }
}

void DenseMap::add(Cloud::ConstPtr world_cloud, double stamp,
                   const Eigen::Vector3f& origin) {
  if (!world_cloud || world_cloud->empty()) return;
  const bool do_carve = origin.allFinite();
  std::lock_guard<std::mutex> lock(this->mtx_);
  for (const auto& p : world_cloud->points) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
    Voxel& v = this->grid_[this->key(p.x, p.y, p.z)];
    v.sx += p.x; v.sy += p.y; v.sz += p.z; v.si += p.intensity;
    ++v.hits;
    if (v.scans == 0 || stamp != v.last_stamp) { ++v.scans; v.last_stamp = stamp; }
    if (do_carve) {
      const Eigen::Vector3f ep(p.x, p.y, p.z);
      const Eigen::Vector3f ray = ep - origin;
      const float len = ray.norm();
      if (len > carve_margin_ + leaf_) {   // carve up to margin short of the hit
        this->carve(origin, origin + ray * ((len - carve_margin_) / len));
      }
    }
  }
}

Cloud::Ptr DenseMap::build(int min_scans) const {
  return this->build(min_scans, std::numeric_limits<float>::infinity());
}

Cloud::Ptr DenseMap::build(int min_scans, float max_free_ratio) const {
  auto out = std::make_shared<Cloud>();
  std::lock_guard<std::mutex> lock(this->mtx_);
  out->reserve(this->grid_.size());
  const std::uint32_t thresh = static_cast<std::uint32_t>(min_scans < 1 ? 1 : min_scans);
  for (const auto& [k, v] : this->grid_) {
    if (v.hits == 0) continue;                         // pure free-space voxel
    if (v.scans < thresh) continue;                    // transient (fast mover)
    const float total = static_cast<float>(v.hits + v.pass);
    if (total > 0.f && (static_cast<float>(v.pass) / total) > max_free_ratio)
      continue;                                        // seen-through too often => dynamic
    PointType pt;
    const float inv = 1.f / static_cast<float>(v.hits);
    pt.x = v.sx * inv; pt.y = v.sy * inv; pt.z = v.sz * inv;
    pt.intensity = v.si * inv;
    out->push_back(pt);
  }
  out->width = out->size();
  out->height = 1;
  out->is_dense = true;
  return out;
}

bool DenseMap::save(const std::string& path, int min_scans, float max_free_ratio,
                    std::string& msg) const {
  Cloud::Ptr m = this->build(min_scans, max_free_ratio);
  if (m->empty()) { msg = "filtered map empty (min_scans=" + std::to_string(min_scans) + ")"; return false; }
  if (pcl::io::savePCDFileBinary(path, *m) != 0) { msg = "savePCDFileBinary failed: " + path; return false; }
  char buf[64]; std::snprintf(buf, sizeof(buf), "%.2f", max_free_ratio);
  msg = "saved " + std::to_string(m->size()) + " pts (min_scans=" +
        std::to_string(min_scans) + ", max_free_ratio=" + buf + ") to " + path;
  return true;
}

std::size_t DenseMap::voxelCount() const {
  std::lock_guard<std::mutex> lock(this->mtx_);
  return this->grid_.size();
}

void DenseMap::clear() {
  std::lock_guard<std::mutex> lock(this->mtx_);
  this->grid_.clear();
}

} // namespace dlio
