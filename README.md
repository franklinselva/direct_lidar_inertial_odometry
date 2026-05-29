# Direct LiDAR-Inertial Odometry: Lightweight LIO with Continuous-Time Motion Correction

#### [[ IEEE ICRA ](https://ieeexplore.ieee.org/document/10160508)] [[ arXiv ](https://arxiv.org/abs/2203.03749)] [[ Video ](https://www.youtube.com/watch?v=4-oXjG8ow10)] [[ Presentation ](https://www.youtube.com/watch?v=Hmiw66KZ1tU)]

DLIO is a new lightweight LiDAR-inertial odometry algorithm with a novel coarse-to-fine approach in constructing continuous-time trajectories for precise motion correction. It features several algorithmic improvements over its predecessor, [DLO](https://github.com/vectr-ucla/direct_lidar_odometry), and was presented at the IEEE International Conference on Robotics and Automation (ICRA) in London, UK in 2023.

<br>
<p align='center'>
    <img src="./doc/img/dlio.png" alt="drawing" width="720"/>
</p>

## Instructions

> **This fork is a pure C++ library.** ROS has been removed entirely. The odometry
> and mapping engines are driven through a push API and configured with a single
> YAML file via `yaml-cpp`. A real sensor and an offline replay feed the exact same
> calls (`addImu` / `addScan`).

### Sensor Setup & Compatibility
DLIO supports Ouster, Velodyne, Hesai, and Livox LiDARs plus a 6-axis IMU. The caller
fills a `pcl::PointCloud<dlio::Point>` (with per-point time in the `t`/`time`/`timestamp`
field for the chosen sensor) and an `dlio::ImuSample`. Set `sensor` in the config to one
of `ouster | velodyne | hesai | livox`; `unknown` disables motion deskew.

For best performance, extrinsic calibration between the LiDAR/IMU sensors and the robot's
center-of-gravity should be entered into `config/dlio.yaml`, along with IMU intrinsics. The
default ideal values plus the startup calibration procedure are sufficient for testing. The
LiDAR and IMU _must_ be time-synchronized — an integrated-IMU LiDAR (e.g. Ouster) is simplest.

### Dependencies

- CMake >= `3.12.4`
- C++17 compiler
- OpenMP >= `4.5`
- Point Cloud Library >= `1.10`
- Eigen >= `3.3.7`
- yaml-cpp

```sh
sudo apt install libomp-dev libpcl-dev libeigen3-dev libyaml-cpp-dev
```

### Compiling

```sh
git clone <this-repo> dlio && cd dlio
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
ctest --test-dir build       # runs the smoke test
```

This produces `libdlio.so`, the `dlio_live_mapping` example, and an installable
`find_package(dlio)` CMake package.

### Library API

```cpp
#include <dlio/config.hpp>
#include <dlio/lio.hpp>
#include <dlio/map.hpp>
#include <dlio/io.hpp>

dlio::Config cfg = dlio::load_config("config/dlio.yaml");
dlio::LIO lio(cfg);
dlio::Map map(cfg.map.sparseLeafSize);

lio.onKeyframe = [&](const dlio::Keyframe& kf){ map.addKeyframe(kf.cloud); };
lio.onPose     = [&](const dlio::State& s, double stamp){ /* stream pose */ };

// feed sensor data (real-time or replay)
lio.addImu(imu_sample);              // dlio::ImuSample{stamp, ang_vel, lin_accel}
lio.addScan(cloud, stamp);           // pcl::PointCloud<dlio::Point>::ConstPtr

// outputs
map.save("dlio_map.pcd", cfg.map.sparseLeafSize);
dlio::io::write_tum_trajectory("trajectory.txt", lio.getTrajectory(), lio.getTrajectoryStamps());
dlio::io::write_keyframes("keyframes/", lio.getKeyframes());
```

### Example: offline replay

The bundled `dlio_live_mapping` example replays a directory of timestamped PCD scans
(`<stamp>.pcd`) plus an IMU CSV (`stamp,wx,wy,wz,ax,ay,az`) through the push API and writes
the map, TUM trajectory, and keyframes:

```sh
./build/dlio_live_mapping config/dlio.yaml <pcd_dir> <imu.csv> <out_dir>
```

### Test Data
For your convenience, we provide test data [here](https://drive.proton.me/urls/Z83QCWKZWW#bMIqDh02AJZZ) (1.2GB, 1m 13s, Ouster OS1-32) of an aggressive motion to test our motion correction scheme, and [here](https://drive.proton.me/urls/7NQSK9DXJ0#gZ9yjGNrDBgG) (16.5GB, 4m 21s, Ouster OSDome) of a longer trajectory outside with lots of trees. Try these two datasets with both deskewing on and off!

<br>
<p align='center'>
    <img src="./doc/gif/aggressive.gif" alt="drawing" width="720"/>
</p>

## Citation
If you found this work useful, please cite our manuscript:

```bibtex
@article{chen2022dlio,
  title={Direct LiDAR-Inertial Odometry: Lightweight LIO with Continuous-Time Motion Correction},
  author={Chen, Kenny and Nemiroff, Ryan and Lopez, Brett T},
  journal={2023 IEEE International Conference on Robotics and Automation (ICRA)},
  year={2023},
  pages={3983-3989},
  doi={10.1109/ICRA48891.2023.10160508}
}
```

## Acknowledgements

We thank the authors of the [FastGICP](https://github.com/SMRT-AIST/fast_gicp) and [NanoFLANN](https://github.com/jlblancoc/nanoflann) open-source packages:

- Kenji Koide, Masashi Yokozuka, Shuji Oishi, and Atsuhiko Banno, “Voxelized GICP for Fast and Accurate 3D Point Cloud Registration,” in _IEEE International Conference on Robotics and Automation (ICRA)_, IEEE, 2021, pp. 11 054–11 059.
- Jose Luis Blanco and Pranjal Kumar Rai, “NanoFLANN: a C++ Header-Only Fork of FLANN, A Library for Nearest Neighbor (NN) with KD-Trees,” https://github.com/jlblancoc/nanoflann, 2014.

We would also like to thank Helene Levy and David Thorne for their help with data collection.

Many thanks to [@shrijitsingh99](https://github.com/shrijitsingh99) for [porting DLIO to ROS2](https://github.com/vectr-ucla/direct_lidar_inertial_odometry/pull/16)!

## License
This work is licensed under the terms of the MIT license.

<br>
<p align='center'>
    <img src="./doc/img/ucla.png" alt="drawing" width="720"/>
</p>
<p align='center'>
    <img src="./doc/img/trees.png" alt="drawing" width="720"/>
</p>
