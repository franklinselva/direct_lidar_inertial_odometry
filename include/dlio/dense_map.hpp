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

#include <cstdint>
#include <limits>
#include <mutex>
#include <string>
#include <unordered_map>

namespace dlio {

/// Voxel-hash dense map with dynamic-obstacle rejection.
///
/// Two signals per voxel:
///   - hits  : times a point landed in the voxel (occupied observation)
///   - pass  : times a ray to a farther point passed THROUGH the voxel
///             (free observation — only counted when a sensor origin is given)
///   - scans : distinct scans that hit the voxel
///
/// Static surfaces are hit consistently and rarely seen-through (high hits, low
/// pass). A dynamic obstacle — even slow or briefly stationary — occupies a
/// voxel for a while, then leaves; once it vacates, every later ray to the now-
/// visible background carves THROUGH its old voxels, driving `pass` up. So the
/// free-ratio pass/(hits+pass) catches movers that a scan-count threshold
/// alone (which treats a dwelling walker as static) cannot.
///
/// Lightweight Amanatides–Woo voxel ray traversal; no TSDF. Pair with an
/// offline pass for the highest quality.
class DenseMap {
public:
  /// carve_margin: rays are carved only up to this distance (m) BEFORE the hit,
  /// leaving a shell around every surface uncarved — prevents grazing rays and
  /// voxel discretization from eroding static structure.
  /// ground_z: voxels with centroid below this height (world frame) are never
  /// carved — protects the ground plane from grazing-ray erosion.
  explicit DenseMap(float leaf_size = 0.10f, float carve_margin = 0.30f,
                    float ground_z = 0.25f);

  /// Fold one world-frame scan (captured at `stamp`) into the grid. If `origin`
  /// (the sensor world position) is finite, rays origin->point are carved,
  /// enabling free-space dynamic removal; otherwise only hits/scans accrue.
  void add(Cloud::ConstPtr world_cloud, double stamp,
           const Eigen::Vector3f& origin =
               Eigen::Vector3f::Constant(std::numeric_limits<float>::quiet_NaN()));

  /// Centroids of voxels seen in >= min_scans distinct scans (scan-count only).
  Cloud::Ptr build(int min_scans) const;

  /// As above, plus drop voxels whose free-ratio pass/(hits+pass) exceeds
  /// max_free_ratio (occupancy carve — removes slow/stationary dynamics).
  /// max_free_ratio >= 1 disables the carve.
  Cloud::Ptr build(int min_scans, float max_free_ratio) const;

  bool save(const std::string& path, int min_scans, float max_free_ratio,
            std::string& msg) const;

  std::size_t voxelCount() const;
  void clear();

private:
  struct Voxel {
    float sx = 0.f, sy = 0.f, sz = 0.f, si = 0.f;  // running sums
    std::uint32_t hits = 0;                        // occupied observations
    std::uint32_t pass = 0;                        // free (carved-through) observations
    std::uint32_t scans = 0;                       // distinct scans observed
    double last_stamp = 0.0;
  };

  std::int64_t key(float x, float y, float z) const;
  std::int64_t keyIdx(std::int64_t ix, std::int64_t iy, std::int64_t iz) const;
  void carve(const Eigen::Vector3f& origin, const Eigen::Vector3f& endpoint);

  float leaf_;
  float carve_margin_;
  float ground_z_;
  std::unordered_map<std::int64_t, Voxel> grid_;
  mutable std::mutex mtx_;
};

} // namespace dlio
