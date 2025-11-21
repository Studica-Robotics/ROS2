## 1. TF Tree Structure

```text
map                                    [world frame, from slam_toolbox]
 │
 └─(map → odom)────────────────────── published by: slam_toolbox
    │                                  corrects drift, updated ~9–10 Hz
    │
    odom                               [odometry frame, local/drifty]
    │
    └─(odom → base_footprint)────────  published by: ekf_filter_node
       │                               fused wheel odom + IMU, ~10 Hz
       │
       base_footprint                  [robot 2D pose on ground plane]
       │
       └─(base_footprint → base_link)  published by: tf_base_link
          │                            static identity (0,0,0)
          │
          base_link                    [robot body center]
          │
          ├─(base_link → imu_link)──── published by: tf_imu
          │  │                         position: (0.092, -0.062, 0.151) m
          │  │
          │  imu_link                  [IMU frame, /imu frame_id]
          │
          ├─(base_link → laser1_frame) published by: tf_laser1
          │  │                         position: (0.144, 0, 0.02) m, yaw 0°
          │  │
          │  laser1_frame              [front LiDAR, /scan1 frame_id]
          │
          ├─(base_link → laser2_frame) published by: tf_laser2
          │  │                         position: (-0.144, 0, 0.02) m, yaw 180°
          │  │
          │  laser2_frame              [rear LiDAR, /scan2 frame_id]
          │
          └─(base_link → laser)────── published by: tf_laser_merged
             │                         position: (0, 0, 0.02) m, yaw 0°
             │
             laser                     [merged LiDAR frame, /merged_scan]
```

---

## 2. TF Publishers Overview

| Transform                 | Publisher Node          | Launch File              | Type    | Notes / Rate              |
|---------------------------|-------------------------|--------------------------|---------|---------------------------|
| `map → odom`              | `slam_toolbox`          | `mapping2_launch.py`     | dynamic | ~9–10 Hz, drift correction |
| `odom → base_footprint`   | `ekf_filter_node`       | `studica_launch.py`      | dynamic | ~10 Hz, fused odom + IMU   |
| `base_footprint → base_link` | `tf_base_link`      | `studica_launch.py`      | static  | identity, latched         |
| `base_link → imu_link`    | `tf_imu`                | `studica_launch.py`      | static  | IMU mount pose            |
| `base_link → laser1_frame`| `tf_laser1`             | `mapping2_launch.py`     | static  | front LiDAR pose          |
| `base_link → laser2_frame`| `tf_laser2`             | `mapping2_launch.py`     | static  | rear LiDAR pose           |
| `base_link → laser`       | `tf_laser_merged`       | `mapping2_launch.py`     | static  | merged scan pose          |

---

## 3. Sensor Fusion Data Flow

### 3.1 Control + EKF (studica_launch.py)

```text
┌──────────────────────────────────────────────────────────────┐
│                    studica_launch.py                        │
│        Control stack + EKF + core TF                        │
└──────────────────────────────────────────────────────────────┘

                 ┌─────────────────────────┐
                 │   control_server        │  (manual_composition)
                 │  studica_control pkg    │
                 └───────────┬─────────────┘
                             │
                 ┌───────────┴──────────────┐
                 │                          │
                 ▼                          ▼
           ┌──────────┐               ┌──────────┐
           │  /odom   │               │  /imu    │
           └────┬─────┘               └────┬─────┘
                │                          │
   nav_msgs/Odometry              sensor_msgs/Imu
   frame_id: odom                  frame_id: imu_link
   child_frame_id: base_footprint  yaw + yaw rate used by EKF
   x, y, vx used by EKF            (roll/pitch largely ignored)
                │                          │
                └────────┬─────────────────┘
                         │
                         ▼
               ┌───────────────────────┐
               │   ekf_filter_node     │  (robot_localization)
               │                       │
               │ Inputs:               │
               │  - /odom  (x, y, vx)  │
               │  - /imu   (yaw, vyaw) │
               │ Output state: x, y,   │
               │  yaw, vx, vyaw        │
               └──────────┬────────────┘
                          │
              ┌───────────┴────────────┐
              │                        │
              ▼                        ▼
      ┌─────────────────┐      ┌──────────────────────┐
      │/odometry/filtered│      │ TF: odom →          │
      └─────────────────┘      │     base_footprint   │
      nav_msgs/Odometry        └──────────────────────┘
      frame_id: odom
      child_frame_id: base_footprint
      rate: ~10 Hz
```

### 3.2 LiDAR + SLAM (mapping2_launch.py)

```text
┌──────────────────────────────────────────────────────────────┐
│                    mapping2_launch.py                       │
│          LiDAR drivers + merging + SLAM + Foxglove         │
└──────────────────────────────────────────────────────────────┘

   ┌──────────────┐              ┌──────────────┐
   │  ydlidar1    │              │  ydlidar2    │
   │ (front)      │              │ (rear)       │
   └──────┬───────┘              └──────┬───────┘
          │                             │
          ▼                             ▼
    ┌──────────┐                  ┌──────────┐
    │ /scan1   │                  │ /scan2   │
    └────┬─────┘                  └────┬─────┘
         │                             │
         └──────────────┬──────────────┘
                        │
                        ▼
              ┌──────────────────────┐
              │ ros2_laser_scan_     │
              │    _merger           │
              │  /scan1 + /scan2 →   │
              │      /merged_scan    │
              └──────────┬───────────┘
                         │
                         ▼
                   ┌──────────────┐
                   │ /merged_scan │  (sensor_msgs/LaserScan)
                   └──────┬───────┘
                          │ frame_id: laser
                          ▼
                ┌──────────────────────┐
                │    slam_toolbox      │
                │  (sync_slam_*)       │
                │                      │
                │ Inputs:              │
                │  - /merged_scan      │
                │  - TF: map→odom→     │
                │        base_footprint│
                │ Outputs:             │
                │  - /map (Occupancy)  │
                │  - TF: map → odom    │
                └──────────┬───────────┘
                           │
              ┌────────────┴─────────────┐
              │                          │
              ▼                          ▼
        ┌──────────┐           ┌──────────────────┐
        │  /map    │           │ TF: map → odom   │
        └──────────┘           └──────────────────┘
        nav_msgs/OccupancyGrid    dynamic, ~9–10 Hz
        frame_id: map             corrects odom drift
```

---

## 4. End-to-End Pose Flow

- In the **odom frame**:

  ```text
  Pose_odom(base_footprint) = (odom → base_footprint)   [from EKF]
  ```

- In the **map frame**:

  ```text
  Pose_map(base_footprint) = (map → odom) ∘ (odom → base_footprint)
  ```

SLAM never modifies `/odometry/filtered`; it only moves the `odom` frame under `map` using `map → odom`.

---

## 5. Quick Reference

- **Frames**
  - `map` – global, drift-corrected frame from SLAM.
  - `odom` – local odometry frame from EKF (may drift slowly).
  - `base_footprint` – 2D base frame on the ground (no roll/pitch).
  - `base_link` – robot body frame.
  - `imu_link` – IMU sensor frame.
  - `laser1_frame`, `laser2_frame` – front and rear LiDAR frames.
  - `laser` – merged LiDAR frame for `/merged_scan`.

- **Key topics**
  - `/odom` – wheel odometry from `control_server`.
  - `/imu` – IMU data in `imu_link`.
  - `/odometry/filtered` – fused pose from EKF.
  - `/scan1`, `/scan2` – raw LiDAR scans.
  - `/merged_scan` – merged front+rear scan (used by SLAM).
  - `/map` – occupancy grid from SLAM.

- **Key transforms**
  - `odom → base_footprint` – EKF pose.
  - `map → odom` – SLAM correction.
  - `base_footprint → base_link` – static base offset.
  - `base_link → imu_link` – IMU mount.
  - `base_link → laser*` – LiDAR mounts.
