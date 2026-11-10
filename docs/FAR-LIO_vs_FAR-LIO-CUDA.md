# FAR-LIO 与 FAR-LIO-CUDA 对比说明

本文回答选型时最关心的几个问题，并给出面向“无人赛车 + 禾赛 64 线 + Livox 补盲”的结论。

## 1. 两个仓库是什么关系

| | **TUMFTM/FAR-LIO**（官方） | **AventLone/FAR-LIO-CUDA**（第三方） |
|---|---|---|
| 来源 | 慕尼黑工业大学 TUM 车辆技术研究所，A2RL 阿布扎比自动驾驶赛车队实际使用 | 个人维护，README 明确说明是对 FAR-LIO `cuVoxelMap` 的“compact implementation”（精简重写），不是官方分支 |
| 论文 | arXiv 2606.26010（FAR-LIO 原始论文） | 无 |
| 代码规模 | `lib_cpp`（ROS 无关的里程计库 + 状态估计库）+ `ros_packages`（ROS 2 节点）+ 文档 + Docker + 官方测试数据 | 单个 `localization` 包（GICP/LIO/LO 节点 + 测试），约 20 个源文件 |
| 工程成熟度 | 有 CI（Docker build）、release 测试包、文档站、版本 tag | 单次提交（2026-11-06），无 tag/release/测试数据；README 自述 ROS 节点集成“未恢复/未验证” |
| 许可证 | Apache-2.0 | 未声明（package.xml 写 TODO） |

**结论：FAR-LIO-CUDA 不是“FAR-LIO 的 CUDA 版官方替代”，而是一个基于其体素地图/GICP 思路的 GPU 实验实现。**
它的价值在于展示了如何把 GICP 全流程放进 CUDA（含 `SparseVoxel` device API、cuCollections 哈希表、240 字节 reduction 等），
适合学习/借鉴；不适合直接作为赛车定位系统。

## 2. 雷达类型与要求

两者都**不限定雷达品牌/线数**，都只接受 `sensor_msgs/PointCloud2`，本质上要求：

1. x/y/z 坐标；
2. （强烈建议）逐点时间戳，用于运动去畸变；
3. 一帧点云对应一个扫描周期。

差异：

- **FAR-LIO（官方）**：`cloud2eigen` 支持 `t` / `time`（float32，相对秒）、`timestamp`（float64，绝对秒）、
  uint32（纳秒偏移）；本次 ROS 1 移植又补了 Livox 的 `offset_time`。旋转式（Velodyne/禾赛/Ouster/RoboSense）
  和非重复扫描（Livox Avia/Mid360/Airy）都能用。禾赛用 `timestamp`，Livox 用 `offset_time`，均已适配。
- **FAR-LIO-CUDA**：点类型/字段处理更“紧凑”，README 未声明对多厂商时间字段的兼容矩阵；
  其 GICP 体素固定 0.5 m、每体素 36 点、协方差邻居 8/12，属于为 GPU 优化的固定参数，调参空间小于官方版。

## 3. ROS 版本

| | 官方 FAR-LIO | FAR-LIO-CUDA |
|---|---|---|
| 原生 | **ROS 2 Jazzy**（Ubuntu 24.04），`ament_cmake`/colcon | **ROS 2**（ament，`rclcpp`） |
| ROS 1 支持 | 无。本次已移植到 **ROS 1 Noetic**（本目录） | 无；且代码用 C++23 + CUDA 13/CCCL，移植到 Noetic 成本更高 |
| 构建 | Docker（推荐）或源码 colcon；`BUILD_CUDA=ON/OFF` | 纯 CMake + ament；`CMAKE_CUDA_ARCHITECTURES=native` |

## 4. 是否限制雷达数量 / 多雷达怎么办

**两者都是“单帧单点云”接口，没有多雷达参数。**

- 官方 `OdometryPipeline::register_frame(frame, stamp)` 一次只接收一帧点云；
  ROS 2 节点只有一个 `node.input_pointcloud`。
- CUDA 版的 LO/LIO 节点同样单话题。

多雷达的正确做法（本移植已提供实现）：

1. **前端融合（推荐）**：各雷达按标定外参变换到 `base_link`，带逐点时间拼接成一帧，送入**一个** FAR-LIO。
   前提是雷达间时间同步（PTP/PPS）或时间差很小（< 10–20 ms）。
   本移植的 `farlio_odometry_multi_lidar` 节点就是做这件事，支持任意 N 个话题，输出带 `time` 字段的合并点云。
2. **多实例**：每个雷达各跑一个 FAR-LIO，再融合位姿。地图会重影、算力翻倍，除非视场完全不重叠，否则不推荐。

对“禾赛 64 线为主 + Livox 补盲”的建议：

- 以禾赛为**主雷达**，Livox 作为补盲/近距离增强，合并后送一个 FAR-LIO；
- 若时间同步做不好，宁可**只用禾赛**跑里程计，Livox 单独用于障碍物/建图，不要强行合并；
- 合并后点数约 25–40 万/帧，CPU 版约 100–200 ms/帧，10 Hz 实时建议 CUDA。

## 5. 是否包含单独的定位模式

**官方 FAR-LIO：有。** `pipeline.update_map` 控制：

- `true`：建图/里程计模式，在线维护局部地图，从原点出发；
- `false` + `node.input_map`：**纯定位模式**，加载先验 PCD（官方还支持通过 `tum_map_msgs` 服务在线取图），
  只配准不改图，输出全局位姿；需要初始位姿（来自 EKF/`/tf` 或配置）。

本 ROS 1 移植完整保留该模式：`update_map: false` + `.pcd` + `localization.initial_pose`（或 `wait_tf`），
并提供 `~save_map` 服务把建图结果存成 PCD。实测定位模式 ATE 0.468 m（KITTI 04，与建图模式一致）。

**FAR-LIO-CUDA：没有明确的“先验地图定位模式”文档/接口**；其 LO 节点是在线建图式 GICP，
LIO 节点是 GICP + CPU ESKF。要做定位需要自己加地图加载与初值接口。

## 6. 传感器融合（IMU）

- 官方 FAR-LIO：**有完整 LIO**。激光 GICP 位姿 + 100 Hz 运动学 EKF（`tam_state_estimation`）融合 IMU，
  并把 EKF 位姿反馈给配准做初值和去畸变。这是它能在高速赛道稳定的关键。
- FAR-LIO-CUDA：有 `ESKF` + `ImuProcessor`，但为简化版（CPU ESKF），无官方精度报告。

> 本 ROS 1 移植目前是 `gicp_cv`（GICP + 匀速模型，纯激光），**EKF 部分未移植**。
> 需要 IMU 紧耦合时可继续移植 `lib_cpp/state_estimation_cpp`，节点已预留 `gicp_ext`（外部初值）模式。

## 7. 选型结论（针对无人赛车）

| 需求 | 选择 |
|---|---|
| 要长期维护、可复现、有论文/测试数据 | **官方 FAR-LIO** |
| 现在就要在 Ubuntu 20.04 + ROS 1 Noetic 上跑 | **官方 FAR-LIO + 本 ROS 1 移植（CPU）** |
| 要 GPU 实时、且能上 Ubuntu 24.04 / ROS 2 Jazzy | 官方 FAR-LIO Docker 镜像（开箱即用） |
| 只想学习 CUDA GICP 实现 | FAR-LIO-CUDA（参考） |

因此本工作区：
- `src/FAR-LIO`：官方仓库（保留，作为算法上游）；
- `src/farlio_ros1`：ROS 1 全功能移植（里程计 + EKF + 定位 + 多雷达 + 地图服务 + 调试）；
- `FAR-LIO-CUDA`：对比评估完成后已按用户要求从 `src` 删除（本文保留评估结论备查）。
