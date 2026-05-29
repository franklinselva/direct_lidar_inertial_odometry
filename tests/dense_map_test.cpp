/***********************************************************
 * DenseMap dynamic-rejection unit test (no ROS, no bag).  *
 * Covers scan-count filtering AND free-space carving.     *
 ***********************************************************/

#include "dlio/dense_map.hpp"

#include <cmath>
#include <iostream>

using namespace dlio;

static void add(Cloud& c, float x, float y, float z) {
  PointType p; p.x = x; p.y = y; p.z = z; p.intensity = 1.f; c.push_back(p);
}
static bool hasNear(const Cloud& c, float x, float y, float z, float tol = 0.4f) {
  for (const auto& p : c.points)
    if (std::fabs(p.x - x) < tol && std::fabs(p.y - y) < tol && std::fabs(p.z - z) < tol)
      return true;
  return false;
}

int main() {
  // ---- Part A: scan-count filter drops a fast (transient) mover ----
  {
    DenseMap dm(0.5f);
    for (int s = 0; s < 10; ++s) {
      auto cl = std::make_shared<Cloud>();
      for (int i = 0; i < 50; ++i) add(*cl, 2.f + i, 5.f, 0.f);   // static wall
      add(*cl, 100.f + s, 0.f, 0.f);                              // mover: new voxel/scan
      dm.add(cl, 0.1 * s);                                        // no origin => count-only
    }
    const std::size_t all = dm.build(1)->size();   // 50 static + 10 mover voxels
    const std::size_t f3  = dm.build(3)->size();   // 50 static (mover seen 1 scan each)
    std::cout << "[A] build(1)=" << all << " build(3)=" << f3 << std::endl;
    if (all != 60 || f3 != 50) { std::cerr << "FAIL A\n"; return 1; }
  }

  // ---- Part B: free-space carve drops a SLOW/dwelling mover that count keeps ----
  {
    DenseMap dm(0.5f);
    const Eigen::Vector3f origin(0.f, 0.f, 0.f);
    for (int s = 0; s < 10; ++s) {
      auto cl = std::make_shared<Cloud>();
      // wall plane at x=10 (every scan), EXCEPT the y=0 bearing is occluded by
      // the dwelling object while it is present (scans 0..2).
      for (float y = -2.f; y <= 2.f; y += 0.5f) {
        if (s < 3 && std::fabs(y) < 0.25f) continue;  // occluded behind mover
        add(*cl, 10.f, y, 0.f);
      }
      if (s < 3) add(*cl, 5.f, 0.f, 0.f);             // mover dwells 3 scans at (5,0,0)
      dm.add(cl, 0.1 * s, origin);                    // origin => carve
    }
    auto count_only = dm.build(2, std::numeric_limits<float>::infinity());
    auto carved     = dm.build(2, 0.5f);
    const bool m_count = hasNear(*count_only, 5.f, 0.f, 0.f);
    const bool m_carve = hasNear(*carved,     5.f, 0.f, 0.f);
    const bool wall_carve = hasNear(*carved, 10.f, 1.f, 0.f);
    std::cout << "[B] count_only mover=" << m_count << " carved mover=" << m_carve
              << " carved wall=" << wall_carve
              << " (sizes " << count_only->size() << " -> " << carved->size() << ")" << std::endl;
    // scan-count (3 scans >= 2) KEEPS the dweller; carve must DROP it; wall stays.
    if (!m_count) { std::cerr << "FAIL B: count-only should keep dweller\n"; return 1; }
    if (m_carve)  { std::cerr << "FAIL B: carve should drop dweller\n"; return 1; }
    if (!wall_carve) { std::cerr << "FAIL B: carve dropped the wall\n"; return 1; }
  }

  std::cout << "PASS: scan-count drops fast movers; carve drops slow/dwelling movers" << std::endl;
  return 0;
}
