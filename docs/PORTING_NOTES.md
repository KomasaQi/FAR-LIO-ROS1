# ROS 1 移植技术说明（全功能）

上游：`TUMFTM/FAR-LIO`，commit `ef20510`（2026-09-23）。
目标：Ubuntu 20.04 + ROS 1 Noetic + CPU-only（无 CUDA toolkit）。

## 1. 设计原则

- **算法源码零改动**：`lib_cpp/odometry_cpp`、`lib_cpp/state_estimation_cpp` 的 `.cpp/.hpp/.cu` 未修改；
- ROS 接口层重写：两个 ROS 2 节点（`tam_odometry`、`tam_state_estimation_node`）→ roscpp；
- 只对 `lib_cpp/odometry_cpp/CMakeLists.txt` 做构建适配（4 处，`ROS1-PORT` 注释标记）；
- TUM 公共库以 **vendored + 去 ament** 的方式引入，不依赖 `TAM__common` 的 ROS 2 生态；
- Eigen/Boost 冲突用 vendored headers / shim 解决，不修改系统目录。

## 2. 上游改动清单

```diff
lib_cpp/odometry_cpp/CMakeLists.txt
- find_package(Eigen3 3.4 REQUIRED)
+ if(NOT TARGET Eigen3::Eigen) find_package(Eigen3 3.4 REQUIRED) endif()

- find_package(param_management_cpp REQUIRED)
- find_package(tsl_logger_cpp REQUIRED)
+ if(NOT TARGET ...::...) find_package(...) endif()

- FetchContent_Declare(googletest ...)   # 总是下载
+ if(BUILD_TESTING) ... endif()

- FetchContent_Declare(tbb URL oneTBB)
- FetchContent_MakeAvailable(robin_map googletest Sophus tbb)
+ find_package(TBB REQUIRED)             # 用 Noetic 的 libtbb-dev
+ FetchContent_MakeAvailable(robin_map Sophus)
+ if(BUILD_TESTING) FetchContent_MakeAvailable(googletest) endif()
```

新增（非修改）：`src/FAR-LIO/CATKIN_IGNORE`，防止 catkin 爬进上游 40+ 个 ament 包。
Sophus 1.24.6 与 robin-map 1.4.0 通过 `FETCHCONTENT_SOURCE_DIR_*` 指向 `thirdparty/`，**离线可编译**。

## 3. vendored 库

```
src/farlio_ros1/vendor/
├── param_management_cpp/          # TUM param manager（纯 C++17，去 ament）
├── tsl_logger_cpp/                # 时间序列 logger（header-only）
├── tum_types_cpp/                 # TUM 类型定义（header-only，依赖 tsl）
├── tum_helpers_cpp/               # TUM 数学/延迟补偿（只编译 delay_compensation + fir）
├── tum_type_conversions_ros_cpp/  # ROS 1 shim：只保留 ErrorLvl <-> DiagnosticStatus
└── state_estimation/
    ├── state_estimation_constants/  # EKF_2D/EKF_3D/KF_BASE
    ├── state_estimation_base/       # StateEstimationBase 接口
    └── tam_state_estimation/        # EKF/状态机/IMU/车辆模型/参考姿态（header-only）
```

说明：
- `tum_helpers_cpp` 只编译 `delay_compensation.cpp` 与 `discrete_filters/fir.cpp`；
  其余源文件（`curvilinear_cosy.cpp` 等）在 pinned submodule 中调用了被标记
  `[[deprecated("... error ...")]]` 的旧 API，且 EKF 不需要它们。
- `tum_type_conversions_ros_cpp` 的完整实现依赖 autoware/tier4 消息，ROS 1 不需要：
  库内只用到 `error_type_from_diagnostic_level` / `diagnostic_level_from_type`（数值与
  `diagnostic_msgs` 一致，shim 为恒等映射）；节点用到的消息转换在
  `include/farlio_odometry/estimation/ros1_conversions.hpp` 中以 ROS 1 类型重新实现，函数名/语义与上游一致。

## 4. 新增代码

### 4.1 里程计包（`farlio_odometry`）

```
include/farlio_odometry/
├── conversions.hpp                 # 点云/位姿/里程计转换（含 Livox offset_time）
├── yaml_params.hpp                 # YAML→pmg init、传感器配置类型化解析
├── config_types.hpp                # GICP_CV（上游只有 GICP_EXT / ICP_CV）
└── estimation/
    ├── types.hpp                   # 配置/运行时结构（ros::Time 版）
    ├── helper.hpp                  # parse_sensor_configs（YAML 版）
    ├── node.hpp                    # EKF 节点主体（100 Hz、延迟补偿、看门狗、发布）
    ├── main_impl.hpp               # 5 个变体共享入口
    ├── ros1_conversions.hpp        # TUM 类型 <-> ROS 1 消息
    ├── topic_watchdog.hpp          # ros2_watchdog 的 ROS 1 等价实现
    ├── node_monitor.hpp            # NodeMonitor 的 ROS 1 等价实现
    └── tsl_publisher.hpp           # TSL 发布（farlio_msgs/TSLValues/Definition）
src/
├── farlio_odometry_node.cpp        # 里程计/定位节点
├── gicp_cv_instantiation.cpp       # 显式实例化 OdometryPipeline<GICP_CV>
├── multi_lidar_merger.cpp          # 多雷达合并
├── map_server.cpp                  # GetPointCloudMap 地图服务
└── state_estimation_{2d,3d}_*.cpp  # 5 个 EKF 可执行文件
```

### 4.2 消息包（`farlio_msgs`）

| 消息/服务 | 对应上游 | 说明 |
|---|---|---|
| `TUMFloat64PerWheel(.Stamped)` | `tum_msgs/TUMFloat64PerWheel*` | 四轮转速（字段同名） |
| `SteeringReport` | `autoware_auto_vehicle_msgs/SteeringReport` | 前轮转角 |
| `TSLValues` / `TSLDefinition` | `tsl_msgs` | 分类型 name/value 数组 + definition hash |
| `GetPointCloudMap.srv` | `tum_map_msgs/GetPointCloudMap` | 直接返回 `PointCloud2`（不再走临时文件） |

## 5. 参数注入

上游 ROS 2 用 `param_management_ros2_integration`：先声明参数，再用 ROS override 回写。
ROS 1 移植在**构造节点/估计器之前**把 YAML 递归拍平成 `a.b.c`，调用
`tam::pmg::init(vector<pair<string, variant>>)` 写入初始化层；`declare_parameter` 时自动取覆盖值。
`param_management_cpp` 的这套“初始化层”语义是 TUM 官方为“无 ROS 环境”设计的，等价于 ROS override。

EKF 节点的传感器配置（`measurements.*` / `inputs.imus.*`）在 YAML 中按子树解析成类型化 map，
交给 `parse_sensor_configs`（逻辑与上游一致，只看数据来源）。

## 6. ROS 1 侧的行为等价实现

| 上游 | 移植 |
|---|---|
| `TopicWatchdog`（定时检查 + 超时回调 + 恢复回调） | `tam::core::TopicWatchdog`（相同语义，100 Hz 检查） |
| `NodeMonitor`（按 key 汇总最坏等级、causing_key、key/value、`<topic>_status`） | `tam::core::NodeMonitor`（相同话题与字段） |
| `tsl_ros2_publisher_cpp`（definition latched + values 触发） | `farlio_ros1::TslPublisher`（定义变化才重发，10 Hz 值） |
| `tum_map_msgs` 地图服务 + `_ready` 异步换图 | `farlio_msgs/GetPointCloudMap` + `request_map_switch()` |
| ROS 2 参数 on-set 回调 | 100 Hz 参数 hash 检查 + 重新读取 |
| EKF 的 5 个可执行变体 | 5 个独立可执行文件（同名规则） |

## 7. 关键行为修正

**TF 发布权**：上游只有非外部初值模式（`*_cv`/`*_icp`）的里程计节点广播 `odom -> base_link`；
`*_ext` 模式由 EKF 广播。移植最初沿用了“总是广播”，导致 `gicp_ext` 用自己上一帧的 TF 做初值，
诊断报 `large time diff`，EKF 拒绝初始化。现改为与上游一致的 `if constexpr (MODEL != EXTERNALGUESS)`。

**EKF 初始化条件**：`initialize_from_measurements: true` 时，EKF 需要一帧状态为 OK 的
`/core/state/lidar_odometry` + 状态 topic；`gicp_ext` 的第一帧（无 EKF TF）会走
`ModelHandler` 的 CV 回退并保持当前帧时间戳，因此诊断 OK、EKF 可初始化。之后 EKF 发布 TF，
里程计切到外部位姿初值，形成上游的闭环。

## 8. 性能（本机，8 线程 CPU）

| 环节 | 耗时 |
|---|---|
| 点云转换 + 去畸变/预处理 + GICP（124k 点，6–20 iter） | 19–45 ms / 帧 |
| EKF step（100 Hz） | < 1 ms / 周期 |
| TSL 发布 | 10 Hz |
| 双雷达合并（245k 点） | 约 140 ms / 帧（10 Hz 需降采样或 CUDA） |

## 9. 真实数据适配（AVIA/Livox、PIX/Velodyne）

### 9.1 Livox CustomMsg 与时间戳

- 新增 `livox_to_pointcloud`：`livox_ros_driver/CustomMsg` → `PointCloud2`，保留 `offset_time`
  （uint32 ns）供 FAR-LIO 去畸变；`auto_stamp_offset` 可把旧消息时间戳对齐到当前 ROS 时间。
- 新增 `imu_stamp_adapter`：IMU 时间戳同偏移重发（AVIA/CICV bag 的消息时间是 2020 年，
  而 bag clock 是 2026 年；不对齐会导致 EKF 延迟补偿全部判过期、永不初始化）。
- 新增 message-only 包 `vendor/livox_ros_driver`（仅 CustomMsg/CustomPoint，不含硬件驱动），
  md5 与官方驱动一致，可直接订阅 livox bag。

### 9.2 初始位姿与先验地图

- `prepare_prior_map.py`：按轨迹包围盒裁剪 + 体素降采样 + 可选 z 范围，输出 FAR-LIO 可加载的
  `x y z intensity` 二进制 PCD。35 M 点的 HitchOpen 全局图 → 2.1 M 点（0.5 m），加载约 3 s。
- 初始位姿必须取 **bag 起始时刻** 在参考轨迹中的位姿：
  - PIX：`pix_moving.bag` 是 mapping 的后半段，`transformations.pcd` 从 1.5 h 前开始；
    正确初值 = 对 `transformations` 按 bag 起始时间插值（-274.571, -207.086, 0.724,
    roll -0.00852, pitch 0.04002, yaw 2.07583）。
  - AVIA：同一 bag 建图，地图原点即首帧，初值取单位位姿。

### 9.3 初始猜测回退的修正

`ExternalGuess` 模式下若 EKF TF 尚不可用，原实现用 `get_odometry()`（首帧恒为零）作为
`set_pose_model(..., valid=false)` 的输入，会把配置的先验初值覆盖成原点，导致定位模式首帧
直接跳到错误位置。移植改为：

- TF 可用 → 正常设初值；
- TF 不可用且已有上一帧配准结果 → 用上一帧位姿 + CV 预测；
- TF 不可用且尚无结果 → 保留配置的初始位姿。

### 9.4 双 overlay 环境

`scripts/env.sh` 同时 source `farlio_ws` 与 `lio_ws`，并显式恢复 `CMAKE_PREFIX_PATH`、
`ROS_PACKAGE_PATH`、`LD_LIBRARY_PATH`；否则 catkin overlay 会互相覆盖，导致
`fast_livo` 或 `farlio_odometry` 的可执行文件找不到。

## 10. 升级上游

```bash
cd src/FAR-LIO && git pull
git apply ../farlio_ros1/docs/upstream_cmake.patch   # 若上游 CMake 未变可跳过
```

若上游修改了 `Point` / `PipelineConfig` / `TConfig` 字段：
`config_types.hpp` 的 `GICP_CV` 需同步（复制 `GICP_EXT` 并改 `MODEL`）。
若上游修改了 EKF 的 `StateEstimationBase` 接口或参数键，需同步 `estimation/node.hpp` 与
`estimation/helper.hpp`（两者的差异已集中在这两个文件）。
