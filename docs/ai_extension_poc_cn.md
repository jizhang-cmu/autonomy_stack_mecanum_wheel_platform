# AI Extension POC：语义目标到航点导航

本文档说明本仓库中 AI Extensions 的真实工程含义，并给出一个最小可运行 POC：AI 节点根据目标检测结果生成 waypoint，底层 base autonomy 负责路径规划、避障和速度控制。

## 1. AI Extension 的接口契约

本仓库的 AI Extensions 不包含现成 AI 模型。它提供的是 ROS 2 topic 级别的扩展接口：

```text
AI 订阅：相机图像、点云、机器人位姿
AI 发布：/joy + /way_point 或 /goal_point
Base autonomy：负责局部路径、避障、跟踪和 /cmd_vel
```

最小闭环：

```text
AI node
  -> /joy       # 进入 waypoint/autonomy mode
  -> /way_point # 发布 map frame 下的目标点
  -> local_planner
  -> pathFollower
  -> /cmd_vel
```

不要让 AI 直接发布 `/cmd_vel`。AI 只决定“去哪儿”，底层导航栈决定“怎么安全过去”。

## 2. 本 POC 新增内容

新增 ROS 2 Python package：

```text
src/ai_extensions/semantic_waypoint_demo
```

主要文件：

| 文件 | 作用 |
|---|---|
| `semantic_waypoint_demo/semantic_waypoint_node.py` | 主节点：检测结果转 waypoint |
| `semantic_waypoint_demo/detectors.py` | detector backend 接口和 mock detector |
| `launch/semantic_waypoint_demo.launch.py` | POC 启动文件 |
| `package.xml`, `setup.py` | ROS 2 Python package 元数据 |

默认模式是 `mock` detector，不依赖 GPU、不依赖模型、不依赖摄像头，可以先在仿真里验证 `/joy + /way_point` 合约。

## 3. Topic 设计

### 订阅

| Topic | Type | 说明 |
|---|---|---|
| `/state_estimation` | `nav_msgs/msg/Odometry` | 当前机器人 map frame 位姿 |
| `/camera/image/transmitted` | `sensor_msgs/msg/Image` | 可选，base station 解压后的图像 |
| `/camera/image/compressed` | `sensor_msgs/msg/CompressedImage` | 可选，压缩图像 |

默认 `image_transport=none`，因此 mock demo 不需要相机。

### 发布

| Topic | Type | 说明 |
|---|---|---|
| `/joy` | `sensor_msgs/msg/Joy` | 进入 waypoint/autonomy mode |
| `/way_point` | `geometry_msgs/msg/PointStamped` | AI 生成的目标航点 |
| `/speed` | `std_msgs/msg/Float32` | 可选，直接在车上或 add-on 电脑上运行时可用 |
| `/navigation_boundary` | `geometry_msgs/msg/PolygonStamped` | 可选，局部边界 |
| `/ai_extension/semantic_waypoint/target` | `geometry_msgs/msg/PointStamped` | 调试：当前 AI 选中的 waypoint |
| `/ai_extension/semantic_waypoint/detection` | `std_msgs/msg/String` | 调试：JSON 格式检测信息 |

注意：base station 通过 `domain_bridge` 运行时，当前桥接配置只反向桥接 `/way_point`、`/goal_point`、`/joy`。`/speed` 和 `/navigation_boundary` 默认不会从 base station 回到车端。

## 4. Mock Demo 启动

编译：

```bash
cd ~/autonomy_stack_mecanum_wheel_platform
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select semantic_waypoint_demo
source install/setup.bash
```

先启动仿真或真实 base autonomy，确保以下 topic 存在：

```bash
ros2 topic list | grep state_estimation
ros2 topic list | grep way_point
ros2 topic list | grep joy
```

启动 AI POC：

```bash
ros2 launch semantic_waypoint_demo semantic_waypoint_demo.launch.py
```

默认行为：

- mock detector 假设目标在相机画面中心
- 节点根据机器人当前 yaw，在前方 `1.5 m` 投一个 waypoint
- 发布一次 `/joy` 进入 waypoint mode
- 以安全频率持续发布 `/way_point`

可以模拟目标在画面右侧：

```bash
ros2 launch semantic_waypoint_demo semantic_waypoint_demo.launch.py mock_center_x:=0.75
```

此时节点会把 waypoint 投到机器人右前方。

## 5. 真实相机输入

如果 AI 节点在 base station 上运行，并使用 base station 脚本解压图像：

```bash
export ROS_DOMAIN_ID=1
ros2 launch semantic_waypoint_demo semantic_waypoint_demo.launch.py \
  image_transport:=raw \
  image_topic:=/camera/image/transmitted \
  require_image:=true
```

如果直接订阅压缩图像：

```bash
ros2 launch semantic_waypoint_demo semantic_waypoint_demo.launch.py \
  image_transport:=compressed \
  image_topic:=/camera/image/compressed \
  configured_image_width:=640 \
  configured_image_height:=480 \
  require_image:=true
```

当前 POC 不解码图像。真实模型接入时，应在 detector backend 中完成图像解码和推理。

## 6. 模型接入点

真实模型只需要实现一个函数：

```python
def detect(image_msg, image_width, image_height) -> Optional[Detection]:
    return Detection(
        label="chair",
        confidence=0.86,
        center_x=0.42,
        center_y=0.55,
        width=0.20,
        height=0.30,
    )
```

可选 backend 名称已预留：

- `yolo_world`
- `grounding_dino`
- `owl_vit`
- `custom`

这些 backend 目前是 placeholder。填入模型加载和推理逻辑后，返回归一化 bbox 即可。

## 7. 航点生成逻辑

节点把 bbox 中心转换为相对方向：

```text
center_x = 0.5       -> 正前方
center_x < 0.5       -> 左前方
center_x > 0.5       -> 右前方
heading_offset       -> 根据 horizontal_fov_deg 计算
target_distance      -> 在该方向前方投点
```

生成的 waypoint 坐标位于 `map` frame：

```text
waypoint.x = robot.x + target_distance * cos(robot_yaw + heading_offset)
waypoint.y = robot.y + target_distance * sin(robot_yaw + heading_offset)
waypoint.z = robot.z
```

## 8. 参数建议

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `target_distance` | `1.5` | 仿真中目标前进距离 |
| `speed` | `0.25` | 发布到 `/speed` 的速度 |
| `waypoint_update_rate_hz` | `2.0` | 避免 waypoint 高频跳动 |
| `stable_detection_frames` | `2` | 连续检测稳定后再发 waypoint |
| `max_heading_offset_deg` | `45.0` | 限制单次目标方向 |
| `joy_forward_axis` | `1.0` | 模仿 RViz Resume 的 waypoint mode 速度输入 |

真实 Hex Ark 初测建议：

```bash
ros2 launch semantic_waypoint_demo semantic_waypoint_demo.launch.py \
  target_distance:=1.0 \
  speed:=0.20 \
  max_heading_offset_deg:=30.0
```

## 9. Demo 路线建议

第一阶段建议做：

```text
目标物导航：去最近的 X
```

原因：

- 不需要跟踪状态机
- 不需要大模型推理
- 不需要 OCR 或地图语义
- 最快验证 AI extension 的核心接口

后续升级顺序：

1. 目标物导航：检测 X 并靠近
2. 行人跟随：目标 tracking + 连续 waypoint
3. 标识牌/OCR：识别文字，决定岔路方向
4. VLM 自然语言导航：语言目标到航点
5. 语义探索：TARE 探索 + 目标检测 + 停止/靠近

## 10. 验证命令

查看 AI 输出：

```bash
ros2 topic echo /ai_extension/semantic_waypoint/detection
ros2 topic echo /ai_extension/semantic_waypoint/target
ros2 topic echo /way_point
```

确认进入 waypoint mode：

```bash
ros2 topic echo /joy
```

确认底层导航链路：

```bash
ros2 topic echo /path
ros2 topic echo /cmd_vel
```

如果 `/way_point` 有输出但机器人不动，优先检查：

- `/joy` 是否发布
- local planner 是否处于 waypoint/autonomy mode
- `/state_estimation` 是否稳定
- `/registered_scan` 和 `/terrain_map` 是否正常
- 真实 Hex Ark 是否已经接入 `/cmd_vel -> hex_base_bridge`
