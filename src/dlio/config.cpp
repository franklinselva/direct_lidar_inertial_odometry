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

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <stdexcept>

namespace dlio {

namespace {

template <typename T>
T get(const YAML::Node& node, const T& fallback) {
  if (node && !node.IsNull()) {
    try { return node.as<T>(); } catch (const YAML::Exception&) {}
  }
  return fallback;
}

Eigen::Vector3f vec3(const YAML::Node& node, const Eigen::Vector3f& fallback) {
  if (node && node.IsSequence() && node.size() == 3) {
    return Eigen::Vector3f(node[0].as<float>(), node[1].as<float>(), node[2].as<float>());
  }
  return fallback;
}

Eigen::Matrix3f mat3(const YAML::Node& node, const Eigen::Matrix3f& fallback) {
  if (node && node.IsSequence() && node.size() == 9) {
    std::array<float, 9> v{};
    for (int i = 0; i < 9; ++i) v[i] = node[i].as<float>();
    return Eigen::Map<const Eigen::Matrix<float, 3, 3, Eigen::RowMajor>>(v.data());
  }
  return fallback;
}

SensorType parse_sensor(const std::string& s) {
  std::string l = s;
  std::transform(l.begin(), l.end(), l.begin(), ::tolower);
  if (l == "ouster") return SensorType::OUSTER;
  if (l == "velodyne") return SensorType::VELODYNE;
  if (l == "hesai") return SensorType::HESAI;
  if (l == "livox") return SensorType::LIVOX;
  return SensorType::UNKNOWN;
}

Extrinsics::SE3 se3(const YAML::Node& node) {
  Extrinsics::SE3 out;
  out.t = vec3(node["t"], out.t);
  out.R = mat3(node["R"], out.R);
  return out;
}

} // namespace

Config load_config(const std::string& path) {
  YAML::Node root = YAML::LoadFile(path);
  if (!root["dlio"]) {
    throw std::runtime_error("dlio: config file '" + path + "' missing top-level 'dlio' key");
  }
  const YAML::Node n = root["dlio"];

  Config c;

  c.version = get<std::string>(n["version"], c.version);
  c.verbose = get<bool>(n["verbose"], c.verbose);
  c.adaptive = get<bool>(n["adaptive"], c.adaptive);
  c.sensor = parse_sensor(get<std::string>(n["sensor"], "unknown"));

  const YAML::Node pc = n["pointcloud"];
  c.pointcloud.deskew = get<bool>(pc["deskew"], c.pointcloud.deskew);
  c.pointcloud.voxelize = get<bool>(pc["voxelize"], c.pointcloud.voxelize);

  const YAML::Node od = n["odom"];
  c.odom.gravity = get<double>(od["gravity"], c.odom.gravity);
  c.odom.computeTimeOffset = get<bool>(od["computeTimeOffset"], c.odom.computeTimeOffset);

  const YAML::Node prep = od["preprocessing"];
  c.odom.preprocessing.cropBoxSize =
      get<double>(prep["cropBoxFilter"]["size"], c.odom.preprocessing.cropBoxSize);
  c.odom.preprocessing.voxelRes =
      get<double>(prep["voxelFilter"]["res"], c.odom.preprocessing.voxelRes);

  const YAML::Node kf = od["keyframe"];
  c.odom.keyframe.threshD = get<double>(kf["threshD"], c.odom.keyframe.threshD);
  c.odom.keyframe.threshR = get<double>(kf["threshR"], c.odom.keyframe.threshR);

  const YAML::Node sm = od["submap"]["keyframe"];
  c.odom.submap.knn = get<int>(sm["knn"], c.odom.submap.knn);
  c.odom.submap.kcv = get<int>(sm["kcv"], c.odom.submap.kcv);
  c.odom.submap.kcc = get<int>(sm["kcc"], c.odom.submap.kcc);

  const YAML::Node imu = od["imu"];
  c.odom.imu.approximateGravity = get<bool>(imu["approximateGravity"], c.odom.imu.approximateGravity);
  c.odom.imu.calibrateAccel = get<bool>(imu["calibration"]["accel"], c.odom.imu.calibrateAccel);
  c.odom.imu.calibrateGyro = get<bool>(imu["calibration"]["gyro"], c.odom.imu.calibrateGyro);
  c.odom.imu.calibrationTime = get<double>(imu["calibration"]["time"], c.odom.imu.calibrationTime);
  c.odom.imu.bufferSize = get<int>(imu["bufferSize"], c.odom.imu.bufferSize);

  const YAML::Node gicp = od["gicp"];
  c.odom.gicp.minNumPoints = get<int>(gicp["minNumPoints"], c.odom.gicp.minNumPoints);
  c.odom.gicp.kCorrespondences = get<int>(gicp["kCorrespondences"], c.odom.gicp.kCorrespondences);
  c.odom.gicp.maxCorrespondenceDistance =
      get<double>(gicp["maxCorrespondenceDistance"], c.odom.gicp.maxCorrespondenceDistance);
  c.odom.gicp.maxIterations = get<int>(gicp["maxIterations"], c.odom.gicp.maxIterations);
  c.odom.gicp.transformationEpsilon =
      get<double>(gicp["transformationEpsilon"], c.odom.gicp.transformationEpsilon);
  c.odom.gicp.rotationEpsilon = get<double>(gicp["rotationEpsilon"], c.odom.gicp.rotationEpsilon);
  c.odom.gicp.initLambdaFactor = get<double>(gicp["initLambdaFactor"], c.odom.gicp.initLambdaFactor);

  const YAML::Node geo = od["geo"];
  c.odom.geo.Kp = get<double>(geo["Kp"], c.odom.geo.Kp);
  c.odom.geo.Kv = get<double>(geo["Kv"], c.odom.geo.Kv);
  c.odom.geo.Kq = get<double>(geo["Kq"], c.odom.geo.Kq);
  c.odom.geo.Kab = get<double>(geo["Kab"], c.odom.geo.Kab);
  c.odom.geo.Kgb = get<double>(geo["Kgb"], c.odom.geo.Kgb);
  c.odom.geo.abias_max = get<double>(geo["abias_max"], c.odom.geo.abias_max);
  c.odom.geo.gbias_max = get<double>(geo["gbias_max"], c.odom.geo.gbias_max);

  const YAML::Node map = n["map"];
  c.map.waitUntilMove = get<bool>(map["waitUntilMove"], c.map.waitUntilMove);
  c.map.denseFiltered = get<bool>(map["dense"]["filtered"], c.map.denseFiltered);
  c.map.sparseLeafSize = get<double>(map["sparse"]["leafSize"], c.map.sparseLeafSize);

  c.imuCalibrate = get<bool>(n["imu"]["calibration"], c.imuCalibrate);
  const YAML::Node intr = n["imu"]["intrinsics"];
  c.accelBias = vec3(intr["accel"]["bias"], c.accelBias);
  c.gyroBias = vec3(intr["gyro"]["bias"], c.gyroBias);
  c.accelScaleMisalign = mat3(intr["accel"]["sm"], c.accelScaleMisalign);

  const YAML::Node ext = n["extrinsics"];
  c.extrinsics.baselink2imu = se3(ext["baselink2imu"]);
  c.extrinsics.baselink2lidar = se3(ext["baselink2lidar"]);
  c.extrinsics.recompute();

  return c;
}

} // namespace dlio
