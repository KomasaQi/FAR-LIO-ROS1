# 真实数据定位运行记录（AVIA/CICV 与 PIX/CICV）

本文件记录两套真实数据的**先验地图制作 → FAR-LIO 满血定位（GICP + EKF）→ RViz 可视化**流程。
所有脚本都在 `farlio_odometry/scripts/`，配置在 `farlio_odometry/config/`。

---

## 0. 环境（一条命令）

```bash
source ~/lio_ws/src/farlio_ros1/farlio_odometry/scripts/env.sh
```

`env.sh` 做了三件事（缺一不可）：

1. 先 source `farlio_ws`（FAR-LIO 移植）再 source `lio_ws`（FAST-LIVO2、pcl_ros 等）；
2. 重新拼好 `CMAKE_PREFIX_PATH` / `ROS_PACKAGE_PATH`，否则 `rosrun farlio_*` 找不到可执行文件、`fast_livo` 也会消失；
3. 把 `/usr/lib/x86_64-linux-gnu` 放到 `LD_LIBRARY_PATH` 最前（MVS SDK 的旧 libusb 会破坏 PCL）。

---

## 1. AVIA / CICV（Livox Avia + 相机 + IMU）

数据：`/media/komasa/Data/Ubuntu文件备份/rosbags/avia_hk_cicv/cicv_avia.bag`
（133 s，`/livox/lidar` 为 `livox_ros_driver/CustomMsg`，`/livox/imu` 约 200 Hz，`/hk/image_rect`）

### 1.1 用 FAST-LIVO2 建图并保存地图

用的是上游 FAST-LIVO2 的 `mapping_avia_hk_cicv.launch`（同节点、同参数），只把配置里的
`pcd_save/pcd_save_en` 打开（`config/fastlivo_avia_hk_cicv_save.yaml`）：

```bash
./scripts/run_avia_mapping.sh \
  "/media/komasa/Data/Ubuntu文件备份/rosbags/avia_hk_cicv/cicv_avia.bag"
```

- 上位机 FAST-LIVO2 在收到 SIGINT 后调用 `savePCD()`，写出
  `FAST-LIVO2/Log/PCD/all_downsampled_points.pcd`（约 490 MB，32 M 点，`x y z rgb`）；
- 脚本随后调用 `prepare_prior_map.py` 做**裁剪 + 0.5 m 体素降采样**，得到
  `~/lio_ws/data/maps/avia_cicv_local_map.pcd`（约 95 k 点、4 MB）。
  裁剪范围来自实测地图分布：`x[-250,80] y[-60,320] z[-12,20]`（原图有 z 到 -5000 的飞点）。

### 1.2 时间戳适配（关键）

该 bag 的 **消息时间戳是 2020 年**（`1590196350`），而 `rosbag play --clock` 的 `/clock` 是录制时刻
（2026 年）。直接喂给 EKF 会让延迟补偿把每条测量判为"过期"，EKF 永远不初始化
（`status level 2 (large time diff)`）。

处理：`avia_localization.launch` 中启用两个适配节点（首帧自动对齐）：

| 节点 | 作用 |
|---|---|
| `farlio_odometry_livox_to_pointcloud` | `CustomMsg` → `PointCloud2`（保留 `offset_time`，供去畸变），`auto_stamp_offset:=true` |
| `farlio_odometry_imu_stamp_adapter` | `/livox/imu` → `/livox/imu_fixed`，同偏移 |

### 1.3 一键定位 + RViz

```bash
./scripts/run_avia_localization.sh \
  "/media/komasa/Data/Ubuntu文件备份/rosbags/avia_hk_cicv/cicv_avia.bag"
```

RViz 固定坐标系 `camera_init`，显示：先验地图 `/prior_map`、实时点云 `/livox/lidar_points`、
轨迹 `/core/state/lidar_odometry_path`、EKF 位姿 `/core/state/odometry`。
只跑半秒验证可用：`RVIZ=false ./scripts/run_avia_localization.sh <bag> duration:=20`。

---

## 2. PIX / CICV（Velodyne 64 + IMU + 组合导航）

数据：`~/HitchOpen-THICV-Stack/src/launch/simple_racing/maps/Cicv_Map_Outside/pix_moving.bag`
（508 s，`/velodyne_points` 10 Hz 48 k 点，字段含 `time`，`/imu_message` 100 Hz，`/odom_message` 100 Hz）

先验地图：同目录 `GlobalMap.pcd`（35 M 点，`x y z intensity`）+ `transformations.pcd`
（2725 个轨迹位姿，字段 `x y z i roll pitch yaw time`）+ `trajectory.pcd`。

### 2.1 制作 FAR-LIO 可用的先验地图

```bash
python3 ./scripts/prepare_prior_map.py \
  --map  "$HOME/HitchOpen-THICV-Stack/src/launch/simple_racing/maps/Cicv_Map_Outside/GlobalMap.pcd" \
  --trajectory "$HOME/HitchOpen-THICV-Stack/src/launch/simple_racing/maps/Cicv_Map_Outside/transformations.pcd" \
  --out "$HOME/lio_ws/data/maps/pix_cicv_local_map.pcd" \
  --margin 40 --voxel 0.5 --z-min -20 --z-max 20
```

得到 `~/lio_ws/data/maps/pix_cicv_local_map.pcd`：2.14 M 点（0.5 m），约 33 MB；加载约 3 s。

### 2.2 初始位姿（关键，容易踩坑）

`pix_moving.bag` **不是** mapping 的起点，而是那次建图过程的后半段：
`transformations.pcd` 时间范围 `1758617930–1758623650`，bag 从 `1758623410` 开始。
所以初始位姿必须取 **bag 起始时刻** 在地图轨迹里的位姿，而不是 `transformations[0]`：

```
t = 1758623410.58  ->  x=-274.571, y=-207.086, z=0.724,
                       roll=-0.00852, pitch=0.04002, yaw=2.07583 (rad)
```

该值已写入 `config/pix_cicv_localization.yaml` 的 `localization.initial_pose`。
（用 `transformations[0]` 会初始化到 1.5 小时前的另一个位置，GICP 会立刻收敛到错误位置。）

外参：`base_link -> velodyne` 用 identity 静态 TF（该车 LiDAR 与车体坐标一致）。

### 2.3 一键定位 + RViz

```bash
./scripts/run_pix_localization.sh \
  "$HOME/HitchOpen-THICV-Stack/src/launch/simple_racing/maps/Cicv_Map_Outside/pix_moving.bag"
```

RViz 固定坐标系 `map`，显示先验地图、`/velodyne_points`、轨迹、EKF 位姿。

---

## 3. 实测结果（2026-11-10）

| 运行 | 时长 | 参考 | 结果 |
|---|---|---|---|
| PIX 定位（仅 LiDAR，gicp_ext + 定位模式） | 60 s 实车段 | 地图轨迹 `transformations.pcd`（同帧，无需对齐） | **平均 0.028 m，RMSE 0.035 m，最大 0.17 m** |
| PIX 满血（GICP + 3D EKF） | 60 s | 同上 | **平均 0.127 m，RMSE 0.133 m，最大 0.27 m** |
| AVIA 定位（Livox + EKF，全 bag） | 133 s | 与先验地图自身一致性 | 轨迹连续、z 正常（-1.6~1.6 m），EKF 初始化成功、状态 level 0 |
| AVIA 建图（FAST-LIVO2） | 133 s | — | 32.25 M 点 PCD → 95 k 点先验图 |

> PIX 的 `/odom_message` 是 UTM 组合导航输出，与地图帧不同，只用于形状对比；
> 绝对精度用同帧的 `transformations.pcd` 评估（不需要 SE(3) 对齐）。

---

## 4. 常用参数

| 参数 | 位置 | 说明 |
|---|---|---|
| `localization.initial_pose` | `config/*_localization.yaml` | 地图帧下的初始位姿 `[x,y,z,roll,pitch,yaw]` |
| `map.voxel_size` | 同上 | 先验图内部体素（0.5 m 图 + 1.0 m 体素） |
| `estimation_variant` | launch | `3d_nh`（默认，上游同款）/ `3d_kin` / `2d_*` |
| `pipeline_type` | launch | `gicp_ext`（闭环）或 `gicp_cv`（无 EKF 初值） |
| `RVIZ` | 脚本环境变量 | `false` 关闭 RViz |
| `duration` | launch | 回放秒数，`0` = 整包 |

---

## 5. 已知限制

- AVIA 地图来自 FAST-LIVO2 的 RGB 点云，`prepare_prior_map.py` 把 `rgb` 字段按 float 读入
  `intensity`（FAR-LIO 不使用 intensity，仅 x/y/z 生效）；
- AVIA 若换用别的 bag（时间戳正常），把 launch 里的 `auto_stamp_offset` 设为 `false` 即可；
- PIX 的初始位姿依赖 `transformations.pcd` 与 bag 的对应关系；换 bag 时需重新按
  "bag 起始时间在轨迹里插值" 得到初始位姿。
