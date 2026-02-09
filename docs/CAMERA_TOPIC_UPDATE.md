# 相机话题更新说明

## 更新内容

两个相机的话题现在都有明确的区分前缀：

### 1. 世界观察相机 (World View Camera)
**话题前缀**: `world/`

**发布的话题**:
- `world/depth/image_raw` - 深度图像
- `world/depth/camera_info` - 深度相机信息
- `world/color/image_raw` - RGB 彩色图像
- `world/color/camera_info` - 彩色相机信息
- `world/infra1/image_raw` - 左红外图像
- `world/infra1/camera_info` - 左红外相机信息
- `world/infra2/image_raw` - 右红外图像
- `world/infra2/camera_info` - 右红外相机信息
- `world/depth/points` - 点云

**TF 坐标系**:
- `world_view_camera_link`
- `world_view_camera_color_optical_frame`
- `world_view_camera_depth_optical_frame`

**用途**: 固定位置观察整个场景

### 2. 末端手眼相机 (End-Effector Hand-Eye Camera)
**话题前缀**: `ee/` (End-Effector 的缩写)

**发布的话题**:
- `ee/depth/image_raw` - 深度图像
- `ee/depth/camera_info` - 深度相机信息
- `ee/color/image_raw` - RGB 彩色图像
- `ee/color/camera_info` - 彩色相机信息
- `ee/infra1/image_raw` - 左红外图像
- `ee/infra1/camera_info` - 左红外相机信息
- `ee/infra2/image_raw` - 右红外图像
- `ee/infra2/camera_info` - 右红外相机信息
- `ee/depth/points` - 点云

**TF 坐标系**:
- `camera_link`
- `camera_color_optical_frame`
- `camera_depth_optical_frame`

**位置**: 安装在 tool0 上方 12cm，前方 5cm，向下俯视
**用途**: Vision 系统使用，用于目标检测和抓取

## Vision 系统话题流

### 话题订阅和发布流程

```
末端相机 (ee/)
    ↓
ee/depth/image_raw → register_depth.launch.py
ee/color/image_raw → obj_detect.py
ee/color/camera_info → obj_detect.py
    ↓
depth_registered/image_rect → obj_detect.py
    ↓
/detection → det_tf.py
```

### Vision 节点订阅的话题

1. **obj_detect.py**:
   - 订阅: `/ee/color/image_raw`, `/ee/color/camera_info`, `/depth_registered/image_rect`
   - 发布: `/detection`, `/detect_track`

2. **register_depth.launch.py**:
   - 订阅: `/ee/depth/image_raw`, `/ee/depth/camera_info`, `/ee/color/camera_info`
   - 发布: `/depth_registered/image_rect`, `/depth_registered/camera_info`

3. **point_cloud_processor.py**:
   - 订阅: `/ee/depth/points`
   - 发布: `/ee/depth/points_filtered`

4. **det_tf.py**:
   - 订阅: `/detection`
   - 发布: TF (`cube1`, `cube2`, ...)

## 测试验证

```bash
# 1. 启动仿真
ros2 launch ur_bringup simulation.launch.py

# 2. 检查话题（应该看到 world/ 和 ee/ 前缀）
ros2 topic list | grep -E "(world|ee)"

# 预期输出：
# /ee/color/camera_info
# /ee/color/image_raw
# /ee/depth/camera_info
# /ee/depth/image_raw
# /ee/depth/points
# /world/color/camera_info
# /world/color/image_raw
# /world/depth/camera_info
# /world/depth/image_raw
# /world/depth/points

# 3. 查看世界相机图像
ros2 run image_tools showimage --ros-args -r image:=/world/color/image_raw

# 4. 查看手眼相机图像
ros2 run image_tools showimage --ros-args -r image:=/ee/color/image_raw

# 5. 检查 vision 系统
ros2 topic echo /detection
ros2 run image_tools showimage --ros-args -r image:=/detect_track
```

## TF 坐标系

### 世界相机 TF 树
```
world → world_view_camera_joint → world_view_camera_link
    → world_view_camera_color_optical_frame
    → world_view_camera_depth_optical_frame
```

### 手眼相机 TF 树
```
tool0 → camera_joint → camera_link
    → camera_color_optical_frame  ← det_tf.py 使用这个
    → camera_depth_optical_frame
```

## 修改的文件

### URDF 配置
- `src/ur5e_gripper_moveit_config/urdf/ur5e_gripper.urdf.xacro`
  - 世界相机话题前缀: `world/`
  - 手眼相机话题前缀: `ee/`

### Vision 节点
- `src/vision/vision/obj_detect.py`
  - 更新订阅 `/ee/color/image_raw`
  - 更新订阅 `/ee/color/camera_info`

- `src/vision/launch/register_depth.launch.py`
  - 更新订阅 `/ee/depth/image_raw`
  - 更新订阅 `/ee/depth/camera_info`
  - 更新订阅 `/ee/color/camera_info`

- `src/vision/vision/point_cloud_processor.py`
  - 更新订阅 `/ee/depth/points`
  - 更新发布 `/ee/depth/points_filtered`

## 优势

1. **清晰区分**: `world/` 和 `ee/` 前缀让话题用途一目了然
2. **易于扩展**: 未来添加更多相机可以继续使用前缀（如 `wrist/`, `overhead/`）
3. **避免冲突**: 两个相机的话题不会混淆
4. **便于调试**: 可以清楚地知道数据来自哪个相机
