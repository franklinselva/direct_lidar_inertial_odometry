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

#include <string>
#include <vector>

namespace dlio::io {

// TUM format: timestamp tx ty tz qx qy qz qw (one row per pose).
bool write_tum_trajectory(const std::string& path,
                          const std::vector<Pose>& poses,
                          const std::vector<double>& stamps);

// Writes <dir>/keyframe_poses.txt (TUM) and <dir>/keyframe_<i>.pcd per keyframe.
bool write_keyframes(const std::string& dir, const std::vector<Keyframe>& keyframes);

} // namespace dlio::io
