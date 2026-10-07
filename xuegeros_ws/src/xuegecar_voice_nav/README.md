# xuegecar_voice_nav

语音/指令导航（第一步：`goto_node`）。根据地点名查坐标，向 Nav2 发送导航目标。

## 构建

```bash
cd ~/-ros2--main/软件/1.源码/xuegeros_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select xuegecar_voice_nav --symlink-install
source install/setup.bash
```

## 使用

1. 先启动导航（任选其一）：

```bash
ros2 launch xuegecar_navigation2 navigation2_rviz.launch.py map:=/path/to/map.yaml
# 或
ros2 launch xuegecar_navigation2 slam_nav2.launch.py
```

2. 在 RViz 里设初始位姿（2D Pose Estimate），让 `map` 坐标系出现。

3. 启动 goto 节点：

```bash
ros2 run xuegecar_voice_nav goto_node
# 或
ros2 launch xuegecar_voice_nav goto.launch.py
```

4. 发布地点名触发导航：

```bash
ros2 topic pub --once /goto_place std_msgs/msg/String "data: '客厅'"
```

也支持带动词的写法，会自动去掉前缀：`"data: '去客厅'"`。

## 地点坐标

地点名和坐标在 `config/places.yaml` 里手动填（`x/y/yaw` 都是占位值，按实际房间改）：

```yaml
厨房: {x: 1.2, y: -0.8, yaw: 0.0}
客厅: {x: -1.5, y: 1.0, yaw: 3.14}
```

坐标获取方式见该 yaml 文件头部的注释。

## 后续（第二步/第三步）

- 第二步：接语音识别，把语音转成文本。
- 第三步：把文本发布到 `/goto_place`（或把 `_parse_place` 换成 LLM 语义解析）。
