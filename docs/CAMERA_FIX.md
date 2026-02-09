# 相机配置修复说明

## 问题
之前修改后，原来的手眼相机（发布 `depth/image_raw`）不见了，导致 vision 系统无法工作。

## 修复方案

### 相机配置

现在有两个相机，正确配置为：

#### 1. 世界相机 (Scene Overview Camera)
- **名称**: `world_view_camera`
- **位置**: 固定在世界坐标系 `xyz="1.5 0 2"`
- **话题前缀**: `world/`
- **用途**: 观察整个场景的固定视角

**发布的话题**:
- `world/depth/image_raw`
- `world/color/image_raw`
- `world/depth/camera_info`
- `world/color/camera_info`
- `world/depth/points`

#### 2. 末端相机 (Hand-Eye Camera)
- **名称**: `camera` (不是 ee_camera！)
- **父链接**: `tool0`
- **位置**: `xyz="0 0.12 0.05"`（在 tool0 上方 12cm，前方 5cm）
- **朝向**: `rpy="0 -1.5708 1.5708"`（向下看，俯视夹爪）
- **话题前缀**: 无（使用默认话题名）
- **用途**: 手眼相机，vision 系统使用

**发布的话题**:
- `depth/image_raw` ← vision 系统订阅这个
- `color/image_raw` ← vision 系统订阅这个
- `depth/camera_info` ← vision 系统订阅这个
- `color/camera_info` ← vision 系统订阅这个
- `depth/points`

### Vision 系统订阅的话题

`src/vision/vision/obj_detect.py` 订阅：
- `/depth_registered/image_rect` (由 register_depth.launch.py 处理)
- `/color/image_raw` (来自末端相机)
- `/color/camera_info` (来自末端相机)

`register_depth.launch.py` 订阅并处理：
- 输入: `/depth/image_raw` (来自末端相机)
- 输出: `/depth_registered/image_rect` (对齐后的深度图)

## 关键点

### 末端相机名称很重要！
- 相机名称必须是 `camera` 而不是 `ee_camera` 或其他名称
- 这样发布的话题才是 `depth/image_raw` 而不是 `ee_camera/depth/image_raw`
- vision 系统才能正常工作

### 自定义 Gazebo 插件
虽然 `_d435.gazebo.xacro` 会自动添加默认插件，但我们添加了自定义插件来：
1. 确保话题名称正确
2. 覆盖默认的 `plugin_name` 行为
3. 指定正确的 TF optical frame 名称

## 测试验证

```bash
# 1. 启动仿真
ros2 launch ur_bringup simulation.launch.py

# 2. 检查话题
ros2 topic list | grep -E "(world|depth|color)"

# 应该看到：
# world/depth/image_raw
# world/color/image_raw
# depth/image_raw
# color/image_raw

# 3. 查看 vision 系统
ros2 topic echo /detection

# 4. 查看检测结果图像
ros2 run image_tools showimage --ros-args -r image:=/detect_track
```

## 配置文件位置

所有相机配置在：
```
src/ur5e_gripper_moveit_config/urdf/ur5e_gripper.urdf.xacro
```

### 世界相机配置 (第 16-48 行)
```xml
<xacro:sensor_d435 parent="world" name="world_view_camera" ...>
    <origin xyz="1.5 0 2" rpy="0 0.785398163 3.1415926"/>
</xacro:sensor_d435>
```

### 末端相机配置 (第 63-75 行)
```xml
<xacro:sensor_d435 parent="tool0" name="camera" ...>
    <origin xyz="0 0.12 0.05" rpy="0 -1.5708 1.5708"/>
</xacro:sensor_d435>
```

**重要**: 末端相机的 `name` 必须是 `"camera"`，这样话题前缀才是空的，发布到默认话题。
