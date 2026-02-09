# Camera Setup Documentation

## Overview
The UR5e robot is equipped with two RealSense D435 cameras:

1. **World View Camera** - Static camera mounted on world frame for scene overview
2. **End-Effector Camera (Hand-Eye)** - Camera mounted on robot end-effector (above gripper, facing down)

## Camera Specifications

### World View Camera
- **Name**: `world_view_camera`
- **Position**: Fixed at `xyz="1.5 0 2"` with orientation `rpy="0 0.785 3.14"`
- **Parent Link**: `world`
- **Topic Prefix**: `world/`
- **Purpose**: Scene overview from fixed position

### End-Effector Camera (Hand-Eye)
- **Name**: `camera` (IMPORTANT: must be "camera" for vision system compatibility)
- **Position**: Mounted on `tool0` at `xyz="0 0.12 0.05"`
  - 12cm above tool0 (Y axis)
  - 5cm forward (Z axis)
- **Orientation**: `rpy="0 -1.5708 1.5708"` (facing downward)
- **Parent Link**: `tool0`
- **Topic Prefix**: None (uses default topics)
- **Purpose**: Hand-eye camera for object detection and grasping

## Published Topics

### World Camera Topics (with `world/` prefix)
| Topic | Type | Description |
|-------|------|-------------|
| `world/depth/image_raw` | sensor_msgs/Image | Depth image |
| `world/depth/camera_info` | sensor_msgs/CameraInfo | Depth camera info |
| `world/color/image_raw` | sensor_msgs/Image | RGB color image |
| `world/color/camera_info` | sensor_msgs/CameraInfo | Color camera info |
| `world/infra1/image_raw` | sensor_msgs/Image | Left infrared image |
| `world/infra1/camera_info` | sensor_msgs/CameraInfo | Left IR camera info |
| `world/infra2/image_raw` | sensor_msgs/Image | Right infrared image |
| `world/infra2/camera_info` | sensor_msgs/CameraInfo | Right IR camera info |
| `world/depth/points` | sensor_msgs/PointCloud2 | Point cloud |

### End-Effector Camera Topics (with `ee/` prefix)
| Topic | Type | Description |
|-------|------|-------------|
| `ee/depth/image_raw` | sensor_msgs/Image | Depth image |
| `ee/depth/camera_info` | sensor_msgs/CameraInfo | Depth camera info |
| `ee/color/image_raw` | sensor_msgs/Image | RGB color image |
| `ee/color/camera_info` | sensor_msgs/CameraInfo | Color camera info |
| `ee/infra1/image_raw` | sensor_msgs/Image | Left infrared image |
| `ee/infra1/camera_info` | sensor_msgs/CameraInfo | Left IR camera info |
| `ee/infra2/image_raw` | sensor_msgs/Image | Right infrared image |
| `ee/infra2/camera_info` | sensor_msgs/CameraInfo | Right IR camera info |
| `ee/depth/points` | sensor_msgs/PointCloud2 | Point cloud |

## Usage

### Starting the Simulation
```bash
ros2 launch ur_bringup simulation.launch.py
```

### Verifying Camera Topics
```bash
# Run the verification script
./src/ur_bringup/scripts/verify_cameras.sh

# Or manually check topics
ros2 topic list | grep -E "(world|depth|color|infra)"

# View camera images
ros2 run image_tools showimage --ros-args -r image:=world/color/image_raw
ros2 run image_tools showimage --ros-args -r image:=color/image_raw
```

### Vision System Integration
The vision system (`vision` package) subscribes to the end-effector camera topics with `ee/` prefix:
- `ee/depth/image_raw`
- `ee/color/image_raw`
- `ee/depth/camera_info`
- `ee/color/camera_info`

The vision nodes automatically handle these topics:
- `obj_detect.py` → subscribes to `ee/color/image_raw`, `ee/color/camera_info`
- `register_depth.launch.py` → subscribes to `ee/depth/image_raw`, `ee/depth/camera_info`, `ee/color/camera_info`
- `point_cloud_processor.py` → subscribes to `ee/depth/points`

To switch to the world camera, you can remap the topics:
```bash
ros2 launch vision seg_and_det.launch.py color_image_raw:=world/color/image_raw color_camera_info:=world/color/camera_info
```

## Camera Frames

### World Camera Frames
```
world → world_camera_joint → world_camera_bottom_screw_frame → world_camera_link
    → world_camera_depth_frame → world_camera_depth_optical_frame
    → world_camera_color_frame → world_camera_color_optical_frame
    → world_camera_infra1_frame → world_camera_infra1_optical_frame
    → world_camera_infra2_frame → world_camera_infra2_optical_frame
```

### End-Effector Camera Frames
```
tool0 → ee_camera_joint → ee_camera_bottom_screw_frame → ee_camera_link
    → ee_camera_depth_frame → ee_camera_depth_optical_frame
    → ee_camera_color_frame → ee_camera_color_optical_frame
    → ee_camera_infra1_frame → ee_camera_infra1_optical_frame
    → ee_camera_infra2_frame → ee_camera_infra2_optical_frame
```

## Camera Parameters

Both cameras use the following default parameters:
- **Depth Update Rate**: 30 Hz
- **Color Update Rate**: 30 Hz
- **Infrared Update Rate**: 1 Hz
- **Min Depth**: 0.2 m
- **Max Depth**: 10.0 m
- **Point Cloud**: Enabled
- **Resolution**: 1280x720

## Notes

1. **TF Frames**: The end-effector camera moves with the robot arm, while the world camera remains fixed
2. **Point Cloud**: Both cameras publish point cloud data that can be used for 3D perception
3. **Gazebo Plugin**: Uses `librealsense_gazebo_plugin.so` for realistic camera simulation
4. **Existing Code**: The vision system continues to work without modifications as it uses the default topic names (end-effector camera)

## Troubleshooting

If cameras are not publishing data:
1. Check if Gazebo is running: `ros2 topic list`
2. Verify URDF: `xacro src/ur5e_gripper_moveit_config/urdf/ur5e_gripper.urdf.xacro`
3. Check for plugin errors in Gazebo terminal
4. Verify TF frames: `ros2 run tf2_tools view_frames`
