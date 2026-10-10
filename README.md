# ris-ctrl

轻量级 ROS 2 ↔ PX4 控制桥接系统，支持 Gazebo 和 AirSim 仿真环境。

## 📋 项目概述

ris-ctrl 提供了一套完整的无人机仿真控制解决方案，包括：
- PX4 固件与 ROS 2 之间的双向通信桥接
- AirSim 仿真环境的传感器数据接口
- 实时控制和任务级控制两种模式
- 完整的传感器数据流（激光雷达、双目相机、深度相机、里程计）

## 🏗️ 系统架构

```
┌─────────────┐     ┌──────────────┐     ┌─────────────┐
│   AirSim    │───▶ │ airsim_bridge│───▶│  ROS 2 Topics│
│  Simulator  │     │              │     │             │
└─────────────┘     └──────────────┘     └──────┬──────┘
                                                │
┌─────────────┐     ┌──────────────┐            │
│  PX4 SITL   │◀───▶│  px4_bridge  │◀─────────┘
│   + XRCE    │     │              │
└─────────────┘     └──────────────┘
```

## 📦 ROS 2 包结构

### 1. `px4_bridge` - PX4 控制桥接
主要的控制桥接节点，提供 PX4 与 ROS 2 之间的双向通信。

### 2. `px4_bridge_msgs` - 消息定义
自定义的 ROS 2 消息和服务类型。

### 3. `airsim_bridge` - AirSim 传感器桥接
从 AirSim 获取传感器数据并发布到 ROS 2。

### 4. `px4_sitl_gz_launch` - Gazebo 启动配置
PX4 SITL + Gazebo 仿真环境的启动文件。

---

## 🚀 快速启动指南

### 环境要求

- Ubuntu 22.04
- ROS 2 Humble
- PX4 固件（SITL 模式）
- AirSim（Windows 或 Linux）
- Python 3.10+

### 编译项目

```bash
cd ~/1_ws/ris-ctrl
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

---

## 🎮 AirSim 仿真完整启动流程

### 终端 1: 启动 AirSim
```bash
# Windows 上启动 AirSim.exe
# 确保 settings.json 已配置好 PX4 车辆和传感器
```

### 终端 2: 启动 PX4 SITL
```bash
cd ~/PX4-Autopilot
make px4_sitl_default none_iris
```

### 终端 3: 启动 MicroXRCE Agent
```bash
MicroXRCEAgent udp4 -p 8888
```

### 终端 4: 启动激光雷达数据桥接
```bash
cd ~/1_ws/ris-ctrl
source install/setup.bash
python3 src/airsim_bridge/src/getLidarData.py
```

### 终端 5: 启动相机数据桥接
```bash
cd ~/1_ws/ris-ctrl
source install/setup.bash
python3 src/airsim_bridge/src/getCameraData.py
```

### 终端 6: 启动 PX4 控制桥接
```bash
cd ~/1_ws/ris-ctrl
source install/setup.bash
ros2 launch px4_bridge bridge.launch.py
```

---

## 📡 ROS 2 接口说明

### 🔵 订阅的话题（输入到 px4_bridge）

#### 实时控制接口
- **`/px4_bridge/in/realtime_control`** (`px4_bridge_msgs/msg/RealtimeControl`)
  
  高频实时控制指令（100Hz），用于路径跟踪、速度控制等。
  
  ```python
  # 消息字段
  float64 vx          # 期望速度 X (m/s, NED 坐标系)
  float64 vy          # 期望速度 Y (m/s)
  float64 vz          # 期望速度 Z (m/s)
  float64 yaw_rate    # 偏航角速率 (rad/s)
  ```

#### 任务级控制接口
- **`/px4_bridge/in/task`** (`px4_bridge_msgs/msg/TaskCommand`)
  
  任务级控制指令，支持起飞、降落、航点等高级命令。
  
  ```python
  # 消息字段
  string action       # 动作类型: "arm", "disarm", "takeoff", "land", "goto", "loiter"
  float64 x           # 目标位置 X (m, ENU 坐标系)
  float64 y           # 目标位置 Y (m)
  float64 z           # 目标位置 Z (m)
  float64 yaw         # 目标偏航角 (rad)
  float64 speed       # 飞行速度 (m/s)
  string detail       # 附加参数 (JSON 格式)
  ```

### 🟢 发布的话题（来自 airsim_bridge 和 px4_bridge）

#### 来自 AirSim 的传感器数据

**激光雷达**
- **`/lidar/points_body`** (`sensor_msgs/msg/PointCloud2`)
  - 机体坐标系下的点云数据
  - 坐标系: FLU (前-左-上)
  
- **`/lidar/points_world`** (`sensor_msgs/msg/PointCloud2`)
  - 世界坐标系下的点云数据
  - 已根据位姿转换到全局坐标系

**相机图像**（30Hz）
- **`/camera/left/image_raw`** (`sensor_msgs/msg/Image`)
  - 左目相机图像 (640×480, BGR8)
  
- **`/camera/right/image_raw`** (`sensor_msgs/msg/Image`)
  - 右目相机图像 (640×480, BGR8)
  
- **`/camera/depth/image_raw`** (`sensor_msgs/msg/Image`)
  - 深度图像 (640×480, 32FC1, 单位: 米)

**里程计**（100Hz）
- **`/drone260/odom`** (`nav_msgs/msg/Odometry`)
  - 无人机位姿和速度
  - 坐标系: ENU (东-北-上)

#### 来自 PX4 的状态反馈

- **`/px4_bridge/out/state`** (`px4_bridge_msgs/msg/VehicleState`)
  - 飞行器完整状态（位置、速度、姿态等）
  
- **`/px4_bridge/out/task_feedback`** (`px4_bridge_msgs/msg/TaskFeedback`)
  - 任务执行状态反馈

---

## 🛠️ 常用操作示例

### 1. 解锁并起飞到 2 米高度

```bash
ros2 topic pub --once /px4_bridge/in/task px4_bridge_msgs/msg/TaskCommand \
  "{action: 'arm'}"

ros2 topic pub --once /px4_bridge/in/task px4_bridge_msgs/msg/TaskCommand \
  "{action: 'takeoff', z: 2.0}"
```

### 2. 前往指定位置

```bash
ros2 topic pub --once /px4_bridge/in/task px4_bridge_msgs/msg/TaskCommand \
  "{action: 'goto', x: 10.0, y: 5.0, z: 3.0, yaw: 1.57, speed: 2.0}"
```

### 3. 悬停

```bash
ros2 topic pub --once /px4_bridge/in/task px4_bridge_msgs/msg/TaskCommand \
  "{action: 'loiter'}"
```

### 4. 降落

```bash
ros2 topic pub --once /px4_bridge/in/task px4_bridge_msgs/msg/TaskCommand \
  "{action: 'land'}"
```

### 5. 实时速度控制

```bash
# 以 1 m/s 向前飞行
ros2 topic pub /px4_bridge/in/realtime_control px4_bridge_msgs/msg/RealtimeControl \
  "{vx: 1.0, vy: 0.0, vz: 0.0, yaw_rate: 0.0}"
```

---

## 📊 监控和调试

### 查看所有话题
```bash
ros2 topic list
```

### 监听状态反馈
```bash
ros2 topic echo /px4_bridge/out/state
```

### 查看图像帧率
```bash
ros2 topic hz /camera/left/image_raw
```

### 查看点云数据
```bash
ros2 topic echo /lidar/points_world --no-arr
```

### 使用 RViz2 可视化
```bash
rviz2
# 添加以下显示:
# - PointCloud2: /lidar/points_world
# - Image: /camera/left/image_raw
# - Odometry: /drone260/odom
```

---

## 🔧 配置文件

### AirSim 配置
位置: `%USERPROFILE%\Documents\AirSim\settings.json` (Windows)

关键配置项：
- **车辆类型**: `PX4Multirotor`
- **通信端口**: `TcpPort: 4560`
- **相机位置**: 左目 Y=-0.1, 右目 Y=0.1, 深度相机 Y=0.0
- **激光雷达**: 16 线, 10 Hz, 100k 点/秒

### PX4 参数
重要参数（在 PX4 控制台或 `settings.json` 中设置）：
```
NAV_RCL_ACT=0    # 遥控信号丢失后的行为
NAV_DLL_ACT=0    # 数据链路丢失后的行为
COM_OBS_AVOID=0  # 禁用避障（如果不需要）
```

---

## ⚙️ 坐标系说明

本系统涉及多个坐标系，理解它们的转换关系很重要：

| 坐标系 | 定义 | 使用位置 |
|--------|------|---------|
| **NED** (北-东-地) | PX4 内部坐标系 | PX4 固件内部 |
| **ENU** (东-北-上) | ROS 标准坐标系 | ROS 2 话题 |
| **FLU** (前-左-上) | 机体坐标系 | 激光雷达本地点云 |

**转换规则**：
- AirSim → ROS: `(x, -y, -z)` 和四元数调整
- 所有 ROS 话题使用 **ENU** 坐标系

---

## 🐛 常见问题

### Q1: 图像帧率低于 30Hz？
**A**: 检查以下几点：
- AirSim 图像分辨率是否过高
- 网络延迟（如果 AirSim 在远程机器）
- CPU 负载是否过高

### Q2: 点云数据为空？
**A**: 
- 确认 AirSim 场景中有可见物体
- 检查激光雷达配置的 FOV 范围
- 确认 `getLidarData.py` 已正常连接

### Q3: PX4 无法解锁？
**A**:
- 检查 MicroXRCE Agent 是否运行
- 确认 PX4 GPS 已定位（SITL 模式下自动）
- 查看 PX4 控制台的错误信息

### Q4: 任务命令无响应？
**A**:
- 确认 `px4_bridge` 已启动
- 检查话题名是否正确: `/px4_bridge/in/task`
- 查看 `task_feedback` 反馈信息

---

## 📚 相关资源

- [PX4 用户手册](https://docs.px4.io/)
- [AirSim 文档](https://microsoft.github.io/AirSim/)
- [ROS 2 Humble 文档](https://docs.ros.org/en/humble/)

---

## 🤝 贡献

基于 [nexus-lite](https://github.com/skyward-cloud/nexus-lite) 项目重构和优化。


## 📮 联系方式

如有问题或建议，请提交 Issue 或 Pull Request。
