# 手眼相机配置说明

## 概述

手眼相机已成功集成到UR5e机械臂+Robotiq夹爪系统中。相机作为独立模块,可以灵活地附加到不同的父link。

## 文件结构

```
src/robotiq_description/urdf/
├── robotiq_2f_85_macro.urdf.xacro    # 夹爪macro (支持可选的手眼相机)
├── hand_eye_camera.urdf.xacro         # 手眼相机独立macro
└── 2f_85.ros2_control.xacro          # ros2_control配置

src/ur5e_gripper_moveit_config/urdf/
├── single_ur5e_gripper.urdf.xacro    # UR5e+夹爪+相机集成
└── ur5e_gripper.urdf.xacro           # 主URDF文件
```

## 配置参数

### robotiq_2f_85_macro.urdf.xacro 参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `add_hand_eye_camera` | boolean | false | 是否添加手眼相机 |
| `camera_parent` | string | `${prefix}robotiq_85_base_link` | 相机父link |
| `camera_origin_xyz` | string | "0.06 0 0.04" | 相机位置 (米) |
| `camera_origin_rpy` | string | "0 -0.785 0" | 相机姿态 (弧度) |

### hand_eye_camera.urdf.xacro 参数

| 参数 | 类型 | 说明 |
|------|------|------|
| `prefix` | string | TF前缀 (如 "hand_eye_") |
| `parent` | string | 父link名称 |
| `use_nominal_extrinsics` | boolean | 是否使用标称外参 |

## 当前配置

### 方案A: 相机集成到夹爪Macro

在 `robotiq_2f_85_macro.urdf.xacro` 中启用:

```xml
<xacro:robotiq_gripper
    name="ur_gripper"
    prefix=""
    parent="tool0"
    add_hand_eye_camera="true"
    camera_parent="robotiq_85_base_link"
    camera_origin_xyz="0.06 0 0.04"
    camera_origin_rpy="0 -0.785 0">
    <origin xyz="0 0 0" rpy="0 0 0" />
</xacro:robotiq_gripper>
```

**优点**: 相机与夹爪成为一体,配置简洁
**缺点**: 相机位置相对于夹爪base,灵活性较低

### 方案B: 相机独立附加到tool0 (当前使用)

在 `single_ur5e_gripper.urdf.xacro` 中:

```xml
<!-- 夹爪 (不包含相机) -->
<xacro:robotiq_gripper
    name="${name}_gripper"
    prefix="${prefix}"
    parent="${prefix}tool0"
    add_hand_eye_camera="false">
    <origin xyz="0 0 0" rpy="0 0 0" />
</xacro:robotiq_gripper>

<!-- 手眼相机 (独立附加) -->
<xacro:hand_eye_camera
    prefix="${prefix}hand_eye_"
    parent="${prefix}tool0">
    <origin xyz="0.05 0 0.05" rpy="0 -0.5 0" />
</xacro:hand_eye_camera>
```

**优点**: 相机独立配置,位置/姿态灵活调整
**缺点**: 配置略复杂

## ROS Topics

手眼相机发布以下topics (前缀: `/hand_eye_`):

| Topic | 类型 | 说明 |
|-------|------|------|
| `/hand_eye_depth/image_raw` | sensor_msgs/Image | 深度图像 |
| `/hand_eye_depth/camera_info` | sensor_msgs/CameraInfo | 深度相机信息 |
| `/hand_eye_color/image_raw` | sensor_msgs/Image | RGB图像 |
| `/hand_eye_color/camera_info` | sensor_msgs/CameraInfo | RGB相机信息 |
| `/hand_eye_depth/points` | sensor_msgs/PointCloud2 | 点云 |
| `/hand_eye_infra1/image_raw` | sensor_msgs/Image | 红外相机1 |
| `/hand_eye_infra2/image_raw` | sensor_msgs/Image | 红外相机2 |

## TF坐标树

```
tool0
└── hand_eye_camera_bottom_screw_frame
    └── hand_eye_camera_link
        ├── hand_eye_camera_depth_frame
        │   └── hand_eye_camera_depth_optical_frame
        ├── hand_eye_camera_color_frame
        │   └── hand_eye_camera_color_optical_frame
        ├── hand_eye_camera_infra1_frame
        │   └── hand_eye_camera_infra1_optical_frame
        └── hand_eye_camera_infra2_frame
            └── hand_eye_camera_infra2_optical_frame
```

## MoveIt配置

`sensors_3d.yaml` 配置:

```yaml
sensors:
  - point_cloud_camera

point_cloud_camera:
    sensor_plugin: occupancy_map_monitor/PointCloudOctomapUpdater
    point_cloud_topic: /hand_eye_depth/points
    max_range: 5.0
    point_subsample: 1
    padding_offset: 0.05
    padding_scale: 1.0
    max_update_rate: 1.0
    filtered_cloud_topic: filtered_cloud
```

## 调整相机位置

修改相机位置和朝向,编辑以下参数:

```xml
<origin xyz="X Y Z" rpy="R P Y" />
```

### 常用配置示例

**向前看 (0度)**:
```xml
<origin xyz="0.05 0 0.05" rpy="0 0 0" />
```

**向下看45度**:
```xml
<origin xyz="0.05 0 0.05" rpy="0 -0.785 0" />
```

**向下看30度** (当前配置):
```xml
<origin xyz="0.05 0 0.05" rpy="0 -0.5 0" />
```

**侧看90度**:
```xml
<origin xyz="0 0.05 0.05" rpy="0 0 1.57" />
```

## 编译

```bash
cd /home/soupcola/ros2_ws
colcon build --packages-select robotiq_description ur5e_gripper_moveit_config --symlink-install
source install/setup.bash
```

## 验证

```bash
# 检查URDF
xacro src/ur5e_gripper_moveit_config/urdf/ur5e_gripper.urdf.xacro

# 启动仿真
ros2 launch ur5e_gripper_moveit_config ur5e_gripper_sim_control.launch.py

# 检查topics
ros2 topic list | grep hand_eye

# 查看相机图像
ros2 run image_tools showimage --ros-args -r image_topic:=/hand_eye_color/image_raw
```
