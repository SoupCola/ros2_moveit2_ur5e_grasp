# 相机配置说明（最终版本）

## 相机配置

系统中有两个 RealSense D435 相机：

### 1. 世界相机（用于抓取检测）
- **名称**: `world_view_camera`
- **位置**: 固定在世界坐标系 `xyz="1.5 0 2"`
- **话题前缀**: 无（空字符串）
- **发布的话题**:
  - `/color/image_raw` ← Vision 系统订阅
  - `/color/camera_info` ← Vision 系统订阅
  - `/depth/image_raw` ← Vision 系统订阅
  - `/depth/camera_info`
  - `/depth/points`
- **用途**: 抓取检测任务
- **TF 坐标系**: `world_view_camera_color_optical_frame`

### 2. 手眼相机（仅发布图像）
- **名称**: `ee_camera`
- **位置**: 安装在 `tool0` 上方 `xyz="0 0.12 0.05"`
- **朝向**: `rpy="0 -1.5708 1.5708"`（向下俯视）
- **话题前缀**: `ee`
- **发布的话题**:
  - `/ee/color/image_raw`
  - `/ee/color/camera_info`
  - `/ee/depth/image_raw`
  - `/ee/depth/camera_info`
  - `/ee/depth/points`
- **用途**: 仅发布图像，不参与抓取检测
- **TF 坐标系**: `ee_camera_color_optical_frame`

## Vision 系统配置

Vision 系统订阅世界相机的默认话题：

### obj_detect.py
```python
self.declare_parameter("image_topic", "/color/image_raw")      # 世界相机
self.declare_parameter("cam_info_topic", "/color/camera_info") # 世界相机
self.declare_parameter("depth_topic", "/depth_registered/image_rect")  # 处理后的深度
```

### register_depth.launch.py
```python
remappings=[('depth/image_rect', '/depth/image_raw'),      # 世界相机
            ('depth/camera_info', '/depth/camera_info'),     # 世界相机
            ('rgb/camera_info', '/color/camera_info')]       # 世界相机
```

### point_cloud_processor.py
```python
self.create_subscription(PointCloud2, '/depth/points', ...)  # 世界相机
```

## 话题列表

### 世界相机（抓取检测使用）
```
/color/image_raw           ← Vision 订阅
/color/camera_info         ← Vision 订阅
/depth/image_raw           ← Vision 订阅
/depth/camera_info
/depth/points              ← Vision 订阅
/depth_registered/image_rect
```

### 手眼相机（仅发布图像）
```
/ee/color/image_raw
/ee/color/camera_info
/ee/depth/image_raw
/ee/depth/camera_info
/ee/depth/points
```

## 测试方法

```bash
# 启动仿真（需要重启 Gazebo 才能使用新配置）
ros2 launch ur_bringup simulation.launch.py

# 检查话题
ros2 topic list | grep -E "^(color|depth|ee)"

# 查看世界相机图像
ros2 run image_tools showimage --ros-args -r image:=/color/image_raw

# 查看手眼相机图像
ros2 run image_tools showimage --ros-args -r image:=/ee/color/image_raw

# 检查 Vision 系统
ros2 topic echo /detection
```

## 重要说明

1. **必须重启 Gazebo** 才能使新配置生效
2. 世界相机发布到默认话题（无前缀），与 Vision 系统兼容
3. 手眼相机发布到 `ee/` 前缀话题，仅用于查看，不参与抓取检测
4. Vision 系统继续使用默认话题名称，无需额外配置

## 配置文件

所有相机配置在：
```
src/ur5e_gripper_moveit_config/urdf/ur5e_gripper.urdf.xacro
```

- 第 23-33 行：世界相机（`topics_ns=""`）
- 第 42-48 行：手眼相机（`topics_ns="ee"`）
