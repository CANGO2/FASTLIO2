# FASTLIO2 ROS2
- for velodyne or other pointcloud users.
- you dont need livox-related packages!!


- [Topics](#Topics)
- [Prerequisites](#Prerequisites)
- [How to Use](#how-to-use)
- [Acknowledgments](#Acknowledgments)


## Development Environment

| Component   | Version          |
|-------------|------------------|
| **OS**      | Ubuntu 22.04     |
| **ROS**     | Humble Hawksbill     |
| **Eigen3**  | 3.4.0            |
| **Lidar**  | Velodyne-VLP16        |
| **IMU**  | Xsens MTi-610      |

## Prerequisites

```text
pcl
Eigen
sophus
gtsam
```

### Install Sophus
```shell
git clone https://github.com/strasdat/Sophus.git
cd Sophus
git checkout 1.22.10
mkdir build && cd build
cmake .. -DSOPHUS_USE_BASIC_LOGGING=ON
make
sudo make install
```

## How to Use

### 1. FAST-LIO2
```
ros2 launch fastlio2 lio_launch.py
```

### 2. PGO
```
ros2 launch pgo pgo_launch.py
```
### 3. Save Map
```
ros2 service call /pgo/save_maps interface/srv/SaveMaps "{file_path: 'your_save_dir', save_patches: true}"
```

---

### 4. Odometry + Relocalization

1) Launch the Relocalization Node

```shell
ros2 launch localizer localizer_launch.py
ros2 bag play your_bag_file // optional
```

2) Set Initial Relocalization Values

```shell
ros2 service call /localizer/relocalize interface/srv/Relocalize "{"pcd_path": "your_map.pcd", "x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0, "pitch": 0.0, "roll": 0.0}"
```

3) Check Relocalization Result

```shell
ros2 service call /localizer/relocalize_check interface/srv/IsValid "{"code": 0}"
```

---

### 5. Consistent Map Optimization

1) Launch the Map Optimization Node

```shell
ros2 launch hba hba_launch.py
```

2) Call the Optimization Service

```shell
ros2 service call /hba/refine_map interface/srv/RefineMap "{"maps_path": "your maps directory"}"
```

**Note:** If you plan to call the optimization service, you must set `save_patches` to `true` when saving the map.

---

## Acknowledgments

1. [FASTLIO2](https://github.com/hku-mars/FAST_LIO)
2. [BLAM](https://github.com/hku-mars/BALM)
3. [HBA](https://github.com/hku-mars/HBA)

---

## Performance Notes

This code mainly uses `timerCB` to trigger the main function at a fixed frequency.
In ROS2, `timer`, `subscriber`, and `service` callbacks are executed in the same thread. On machines with limited performance, this can cause blocking issues.
To improve performance, it is recommended to use multithreading so that time-consuming callbacks (e.g., `timerCB`) run in a separate thread.
