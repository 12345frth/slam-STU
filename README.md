# Scan Context ROS 2 工作流程

这份仓库提供了一套适用于 ROS 2 TurtleBot3 的 Scan Context 流程，分成两个阶段：

1. 建图时记录数据库
2. 使用地图时做初始位姿定位

核心脚本如下：

- `scan_context_recorder.py`
  - 在建图时记录关键帧
  - 保存 Scan Context 描述子和对应地图位姿
- `scan_context_localizer.py`
  - 读取已经录好的数据库
  - 用实时 `/scan` 做匹配
  - 发布 `/initialpose` 给 AMCL，并在 RViz 中显示结果

---

## 1. 文件说明

- `scan_context_common.py`
  - 公共工具函数
  - 负责描述子、位姿和数据库的读写
- `scan_context_recorder.py`
  - 建图阶段使用的数据库记录器
- `scan_context_localizer.py`
  - 导航阶段使用的定位节点
- `scan_context_lite_node.py`
  - 更早的单文件简化版示例
- `scan_context_map_localizer.py`
  - 旧版实验脚本，基于静态地图合成参考库

正式使用时，建议优先使用：

- `scan_context_recorder.py`
- `scan_context_localizer.py`

---

## 2. 运行前准备

你需要安装：

- ROS 2 Humble
- TurtleBot3 相关包
- `turtlebot3_gazebo`
- `turtlebot3_cartographer`
- `turtlebot3_navigation2`
- `nav2_map_server`

如果系统里没有 PyYAML，可以先安装：

```bash
sudo apt install python3-yaml
```

---

## 3. 推荐目录结构

建议把地图、数据库和脚本放在同一个工作目录里，例如：

```text
~/maps/
  tb3_map.pgm
  tb3_map.yaml
  scan_context/
    descriptors.bin
    poses.csv
    metadata.yaml
  scan_context_recorder.py
  scan_context_localizer.py
  scan_context_common.py
```

其中 `scan_context/` 会由 recorder 自动生成。

---

## 4. 快速开始

如果你已经有现成数据库，可以直接跳到第 6 节启动定位。

如果你还没有数据库，请先按第 5 节重新建图并录库。

---

## 5. 建图并录库

建图时，`scan_context_recorder.py` 需要和 SLAM 同时运行。

它会监听：

- `/scan`
- `map -> base_link` 的 TF

然后在机器人位姿变化足够大时保存关键帧。

### 5.1 启动 Gazebo

推荐先用 TurtleBot3 World 做 SLAM 测试：

```bash
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

如果你只想先验证链路，也可以先用空场景：

```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo empty_world.launch.py
```

### 5.2 启动 SLAM

Humble 下推荐使用 Cartographer：

```bash
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
```

说明：

- 有些 TurtleBot3 版本的 Cartographer launch 会顺手打开 RViz
- 这个 RViz 只是默认可视化窗口，不是必须项
- 你后面做导航时也需要 RViz，但可以复用同一个窗口
- 如果当前窗口已经够用，不需要额外再开一个

### 5.3 启动 Scan Context 记录器

> **重要**：recorder 依赖 `map → base_link` 的 TF 变换，必须等 Gazebo 和 Cartographer 都已启动且机器人模型加载完成后才能运行，否则会因 `base_link does not exist` 而崩溃。启动顺序见 [第 9 节](#9-推荐的完整流程)。

建议在 `~/maps` 目录下运行：

```bash
source /opt/ros/humble/setup.bash
cd ~/maps
python3 scan_context_recorder.py --ros-args \
  -p use_sim_time:=True \
  -p output_dir:=scan_context \
  -p scan_topic:=/scan \
  -p map_frame:=map \
  -p base_frame:=base_link
```

默认关键帧参数：

- 时间间隔：`0.4 s`
- 平移阈值：`0.20 m`
- 角度阈值：`6.0 deg`
- Scan Context 环数：`40`
- Scan Context 扇区数：`120`
- 最小半径：`0.35 m`（保留近场结构）
- 最大半径：`8.0 m`（截断远端噪声）

### 5.4 通过键盘控制机器人走图

另开一个终端：

```bash
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 run turtlebot3_teleop teleop_keyboard
```

建议你：

- 慢慢开出去
- 多转几个角度
- 尽量把整个环境走完整

如果机器人几乎没动，recorder 可能只会记录到很少的 keyframe。

### 5.5 保存地图

建图完成后，保存 `pgm` 和 `yaml`：

```bash
source /opt/ros/humble/setup.bash
ros2 run nav2_map_server map_saver_cli -f ~/maps/tb3_map --ros-args \
  -p map_subscribe_transient_local:=true \
  -p save_map_timeout:=10.0
```

会生成：

- `tb3_map.pgm`
- `tb3_map.yaml`

请确保这两个文件放在同一个目录下。

---

## 6. 使用数据库做定位

数据库和地图准备好后，就可以进入导航和定位阶段。

### 6.1 启动导航

```bash
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=$HOME/maps/tb3_map.yaml
```

这会启动 Nav2 和 AMCL，并加载你保存好的地图。

### 6.2 启动 Scan Context 定位节点

如果数据库目录就是 `~/maps/scan_context`：

```bash
source /opt/ros/humble/setup.bash
cd ~/maps
python3 scan_context_localizer.py --ros-args \
  -p use_sim_time:=True \
  -p database_dir:=scan_context \
  -p scan_topic:=/scan \
  -p top_k_key_candidates:=60 \
  -p key_distance_threshold:=1.2 \
  -p desc_distance_threshold:=0.25
```

或者直接写绝对路径：

```bash
python3 scan_context_localizer.py --ros-args \
  -p use_sim_time:=True \
  -p database_dir:=/home/zhou/maps/scan_context \
  -p scan_topic:=/scan \
  -p top_k_key_candidates:=60 \
  -p key_distance_threshold:=1.2 \
  -p desc_distance_threshold:=0.25
```

这个节点会：

- 加载 `descriptors.bin`
- 加载 `poses.csv`
- 加载 `metadata.yaml`
- 用实时 `/scan` 进行匹配
- 发布 `/initialpose`
- 订阅 `/amcl_pose`
- 发布 RViz marker 到 `/scan_context_markers`

### 6.3 在 RViz 中查看效果

打开 RViz 后，把 `Fixed Frame` 设为 `map`，然后添加这些显示项：

- `LaserScan`，话题 `/scan`
- `PoseWithCovarianceStamped` 或 AMCL 相关显示，话题 `/amcl_pose`
- `MarkerArray`，话题 `/scan_context_markers`

正常情况下你会看到：

- Scan Context 给出的初始位姿
- AMCL 在此基础上继续收敛
- 位姿和地图逐渐对齐

---

## 7. 录库文件说明

`scan_context_recorder.py` 会在 `scan_context/` 中输出三类文件：

- `descriptors.bin`
  - 每个关键帧的 Scan Context 描述子
- `poses.csv`
  - 每个关键帧对应的地图位姿
- `metadata.yaml`
  - 采样阈值、话题名、坐标系、环/扇区参数、统计信息等

这套数据库是后续定位阶段的输入。

另外，`scan_context_recorder.py` 和 `scan_context_localizer.py` 现在都会把同一份运行日志导出到 `logs/` 目录，方便你回放排障。

---

## 8. 常见问题

### 8.1 记录器提示拿不到 TF

**`base_link does not exist`** — Gazebo / Cartographer 尚未就绪，TF 树里还没有 `base_link`。

检查：

- 启动顺序是否正确：Gazebo → Cartographer → recorder（见[第 9 节](#9-推荐的完整流程)）
- SLAM 是否已经启动
- 机器人是否已经开始输出传感器数据

**`Lookup would require extrapolation into the future`** — 这是 `use_sim_time:=True` 下 Gazebo 与 TF 的时钟漂移导致的，scan 时间戳略微超前于 TF 最新数据。当前代码已自动降级为取最新可用 TF（`rclpy.time.Time()`），不会影响建库。如果仍然频繁出现，可适当增大 `tf_lookup_timeout_sec`（默认 0.2s）。

### 8.2 定位节点找不到数据库

请确认 `scan_context/` 目录里至少有：

- `descriptors.bin`
- `poses.csv`
- `metadata.yaml`

### 8.3 AMCL 一直不收敛

可以尝试：

- 让机器人轻微移动一下
- 录库时走得更完整一些
- 检查地图和数据库是否来自同一个环境

### 8.4 定位结果有歧义

在走廊、对称区域、重复结构区域里，这种情况是正常的。

可以尝试：

- 录更多关键帧
- 提高环境覆盖度
- 适当调整阈值

### 8.5 `map_saver_cli` 保存失败

如果你遇到 `Failed to spin map subscription`，通常先检查：

- `cartographer` 是否还在运行
- `/map` 是否真的在发布
- 是否使用了：
  - `map_subscribe_transient_local:=true`
  - `save_map_timeout:=10.0`

---

## 9. 推荐的完整流程

**每个终端独立运行，必须严格按顺序启动，前一步就绪后再执行下一步。**

```
终端 1: Gazebo         → 等待机器人模型加载完成
终端 2: Cartographer     → 等待 SLAM 开始发布 /map 和 TF
终端 3: recorder         → 依赖 base_link TF，切勿提前启动
终端 4: teleop           → 遥控机器人走完全场
终端 5: map_saver        → 保存地图
  ↓ 关闭 Gazebo/Cartographer/recorder/teleop
终端 1: Nav2             → 重新加载保存的地图
终端 2: localizer        → 自动发布 /initialpose 给 AMCL
终端 3: RViz             → 可视化确认收敛
```

### 建图阶段

1. **启动 Gazebo** — 机器人模型和仿真世界
2. **启动 Cartographer** — SLAM 建图（确认 `/map` topic 和 `map → base_link` TF 都已发布）
3. **启动 `scan_context_recorder.py`** — ⚠️ 必须在上面两个都就绪后再启，否则 TF 查找失败
4. **通过 teleop 走完全场** — 慢速、全覆盖、多转角度
5. **保存地图** — `map_saver_cli` 导出 `.pgm` 和 `.yaml`

### 导航阶段

6. **启动 `navigation2.launch.py`** — 加载保存的地图，启动 Nav2 + AMCL
7. **启动 `scan_context_localizer.py`** — 加载数据库，进行全局重定位
8. **在 RViz 中确认** — 观察初始位姿和 AMCL 收敛结果

---

## 10. 下一步建议

如果你想把流程再简化一点，下一步可以继续加一个 launch 文件，把下面这些一次性拉起来：

- Nav2
- recorder
- localizer
- RViz

这样你以后启动会更省事。
