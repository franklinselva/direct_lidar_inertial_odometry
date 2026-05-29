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

#include "dlio/config.hpp"
#include "dlio/types.hpp"

#include <nano_gicp/nano_gicp.h>

#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>

#include <boost/circular_buffer.hpp>

#include <atomic>
#include <condition_variable>
#include <ctime>
#include <functional>
#include <future>
#include <mutex>
#include <thread>
#include <vector>

namespace dlio {

class LIO {
public:
  explicit LIO(const Config& cfg);
  ~LIO();

  void start();

  // Push API (replaces ROS callbacks)
  void addImu(const ImuSample& imu);
  bool addScan(Cloud::ConstPtr scan, double stamp);

  // Programmatic setup (alternative to YAML config).
  // Must be called before any data is fed; throws std::logic_error otherwise.
  void setExtrinsics(const Extrinsics& extrinsics);
  void setImuExtrinsics(const Eigen::Vector3f& t, const Eigen::Matrix3f& R);
  void setLidarExtrinsics(const Eigen::Vector3f& t, const Eigen::Matrix3f& R);
  void setImuIntrinsics(const ImuIntrinsics& intrinsics);
  void setImuCalibration(bool enable);

  Extrinsics getExtrinsics() const;
  ImuIntrinsics getImuIntrinsics() const;
  bool imuCalibrationEnabled() const { return this->imu_calibrate_; }

  // Accessors
  State getState() const;
  Pose getLidarPose() const;
  std::vector<Pose> getTrajectory() const;
  std::vector<double> getTrajectoryStamps() const;
  std::vector<Keyframe> getKeyframes() const;
  bool initialized() const { return this->dlio_initialized; }

  // Live callbacks (replace ROS publishers); all optional
  std::function<void(const State&, double stamp)> onPose;
  std::function<void(Cloud::ConstPtr deskewed_world, const State&, double stamp)> onDeskewedCloud;
  std::function<void(const Keyframe&)> onKeyframe;

private:
  struct ImuMeas {
    double stamp = 0.0;
    double dt = 0.0;
    Eigen::Vector3f ang_vel = Eigen::Vector3f::Zero();
    Eigen::Vector3f lin_accel = Eigen::Vector3f::Zero();
  };

  void getScan(Cloud::ConstPtr scan, double stamp);
  void preprocessPoints();
  void deskewPointcloud();
  void initializeInputTarget();
  void setInputSource();

  void initializeDLIO();

  void getNextPose();
  bool imuMeasFromTimeRange(double start_time, double end_time,
                            boost::circular_buffer<ImuMeas>::reverse_iterator& begin_imu_it,
                            boost::circular_buffer<ImuMeas>::reverse_iterator& end_imu_it);
  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>
    integrateImu(double start_time, Eigen::Quaternionf q_init, Eigen::Vector3f p_init, Eigen::Vector3f v_init,
                 const std::vector<double>& sorted_timestamps);
  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>
    integrateImuInternal(Eigen::Quaternionf q_init, Eigen::Vector3f p_init, Eigen::Vector3f v_init,
                         const std::vector<double>& sorted_timestamps,
                         boost::circular_buffer<ImuMeas>::reverse_iterator begin_imu_it,
                         boost::circular_buffer<ImuMeas>::reverse_iterator end_imu_it);
  void propagateGICP();

  void propagateState();
  void updateState();

  void setAdaptiveParams();

  void computeMetrics();
  void computeSpaciousness();
  void computeDensity();

  ImuSample transformImu(const ImuSample& imu);

  void updateKeyframes();
  void computeConvexHull();
  void computeConcaveHull();
  void pushSubmapIndices(std::vector<float> dists, int k, std::vector<int> frames);
  void buildSubmap(State vehicle_state);
  void buildKeyframesAndSubmap(State vehicle_state);
  void pauseSubmapBuildIfNeeded();
  void emitKeyframe(int idx);

  void debug();

  void ensureNotStarted(const char* what) const;

  Config cfg_;
  std::atomic<bool> data_started_;

  // Flags
  std::atomic<bool> dlio_initialized;
  std::atomic<bool> first_valid_scan;
  std::atomic<bool> first_imu_received;
  std::atomic<bool> imu_calibrated;
  std::atomic<bool> submap_hasChanged;
  std::atomic<bool> gicp_hasConverged;
  std::atomic<bool> deskew_status;
  std::atomic<int> deskew_size;

  // Threads
  std::thread metrics_thread;
  std::thread debug_thread;

  // Trajectory
  std::vector<std::pair<Eigen::Vector3f, Eigen::Quaternionf>> trajectory;
  std::vector<double> trajectory_stamps;
  double length_traversed;
  Eigen::Vector3f length_prev_p_;
  bool length_started_;

  // Keyframes
  std::vector<std::pair<std::pair<Eigen::Vector3f, Eigen::Quaternionf>,
                        Cloud::ConstPtr>> keyframes;
  std::vector<double> keyframe_timestamps;
  std::vector<std::shared_ptr<const nano_gicp::CovarianceList>> keyframe_normals;
  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> keyframe_transformations;
  mutable std::mutex keyframes_mutex;

  // Sensor Type
  SensorType sensor;

  // Preprocessing
  pcl::CropBox<PointType> crop;
  pcl::VoxelGrid<PointType> voxel;

  // Point Clouds
  Cloud::ConstPtr original_scan;
  Cloud::ConstPtr deskewed_scan;
  Cloud::ConstPtr current_scan;

  // Keyframes
  Cloud::ConstPtr keyframe_cloud;
  int num_processed_keyframes;

  pcl::ConvexHull<PointType> convex_hull;
  pcl::ConcaveHull<PointType> concave_hull;
  std::vector<int> keyframe_convex;
  std::vector<int> keyframe_concave;

  // Submap
  Cloud::ConstPtr submap_cloud;
  std::shared_ptr<const nano_gicp::CovarianceList> submap_normals;
  std::shared_ptr<const nanoflann::KdTreeFLANN<PointType>> submap_kdtree;

  std::vector<int> submap_kf_idx_curr;
  std::vector<int> submap_kf_idx_prev;

  bool new_submap_is_ready;
  std::future<void> submap_future;
  std::condition_variable submap_build_cv;
  bool main_loop_running;
  std::mutex main_loop_running_mutex;

  // Timestamps
  double scan_header_stamp;
  double scan_stamp;
  double prev_scan_stamp;
  double scan_dt;
  std::vector<double> comp_times;
  std::vector<double> imu_rates;
  std::vector<double> lidar_rates;

  double first_scan_stamp;
  double elapsed_time;

  // GICP
  nano_gicp::NanoGICP<PointType, PointType> gicp;
  nano_gicp::NanoGICP<PointType, PointType> gicp_temp;

  // Transformations
  Eigen::Matrix4f T, T_prior, T_corr;
  Eigen::Quaternionf q_final;

  Eigen::Vector3f origin;
  Extrinsics extrinsics;

  // IMU
  double imu_stamp;
  double first_imu_stamp;
  double prev_imu_stamp;
  double imu_dp, imu_dq_deg;

  ImuMeas imu_meas;

  boost::circular_buffer<ImuMeas> imu_buffer;
  std::mutex mtx_imu;
  std::condition_variable cv_imu_stamp;

  static bool comparatorImu(ImuMeas m1, ImuMeas m2) {
    return (m1.stamp < m2.stamp);
  }

  // Geometric Observer
  struct Geo {
    bool first_opt_done;
    std::mutex mtx;
    double dp;
    double dq_deg;
    Eigen::Vector3f prev_p;
    Eigen::Quaternionf prev_q;
    Eigen::Vector3f prev_vel;
  }; mutable Geo geo;

  State state;

  Pose lidarPose;
  Pose imuPose;

  // Metrics
  struct Metrics {
    std::vector<float> spaciousness;
    std::vector<float> density;
  }; Metrics metrics;

  std::string cpu_type;
  std::vector<double> cpu_percents;
  clock_t lastCPU, lastSysCPU, lastUserCPU;
  int numProcessors;

  // Parameters (populated from cfg_ in constructor)
  std::string version_;
  int num_threads_;
  bool verbose;

  bool deskew_;
  double gravity_;
  bool time_offset_;
  bool adaptive_params_;

  double keyframe_thresh_dist_;
  double keyframe_thresh_rot_;

  int submap_knn_;
  int submap_kcv_;
  int submap_kcc_;

  bool densemap_filtered_;
  bool wait_until_move_;

  double crop_size_;

  bool vf_use_;
  double vf_res_;

  bool imu_calibrate_;
  bool calibrate_gyro_;
  bool calibrate_accel_;
  bool gravity_align_;
  double imu_calib_time_;
  int imu_buffer_size_;
  Eigen::Matrix3f imu_accel_sm_;

  int gicp_min_num_points_;
  int gicp_k_correspondences_;
  double gicp_max_corr_dist_;
  int gicp_max_iter_;
  double gicp_transformation_ep_;
  double gicp_rotation_ep_;
  double gicp_init_lambda_factor_;

  double geo_Kp_;
  double geo_Kv_;
  double geo_Kq_;
  double geo_Kab_;
  double geo_Kgb_;
  double geo_abias_max_;
  double geo_gbias_max_;
};

} // namespace dlio
