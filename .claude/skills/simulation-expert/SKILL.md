---
name: simulation-expert
description: Expert in simulating autonomous robots with ROS 2 and Gazebo Sim (Ignition). Handles asset preparation, sensor integration, bridging, and launch configuration.
---

# Simulation Expert Skill

This skill provides detailed instructions for setting up and managing Gazebo Sim environments for ROS 2.

## 1. Asset Preparation (URDF & Plugins)

### Plugin Namespace Difference
- **Gazebo Fortress (Humble):** `ignition::gazebo::systems::...` | filename: `ignition-gazebo-...`
- **Gazebo Harmonic (Jazzy):** `gz::sim::systems::...` | filename: `gz-sim-...`

### Diff Drive Plugin Example (Harmonic/Jazzy)
```xml
<gazebo>
  <plugin filename="gz-sim-diff-drive-system" name="gz::sim::systems::DiffDrive">
    <left_joint>left_wheel_joint</left_joint>
    <right_joint>right_wheel_joint</right_joint>
    <wheel_separation>1.25</wheel_separation>
    <wheel_radius>0.3</wheel_radius>
    <topic>cmd_vel</topic>
    <odom_topic>odom</odom_topic>
    <tf_topic>tf</tf_topic>
    <frame_id>odom</frame_id>
    <child_frame_id>base_link</child_frame_id>
    <odom_publisher_frequency>50</odom_publisher_frequency>
  </plugin>
  
  <plugin filename="gz-sim-joint-state-publisher-system" name="gz::sim::systems::JointStatePublisher">
    <topic>joint_states</topic>
  </plugin>
</gazebo>
```

## 2. Sensor Integration

**CRITICAL:** You MUST add the `gz::sim::systems::Sensors` plugin to the `<world>` or `<robot>` (via `<gazebo>` tag) for sensors to work.

### Lidar Example
```xml
<gazebo reference="lidar_link">
  <sensor name="lidar" type="gpu_lidar">
    <topic>scan</topic>
    <update_rate>10</update_rate>
    <lidar>
      <scan>
        <horizontal>
          <samples>360</samples>
          <resolution>1</resolution>
          <min_angle>-3.14</min_angle>
          <max_angle>3.14</max_angle>
        </horizontal>
      </scan>
      <range>
        <min>0.10</min>
        <max>10.0</max>
      </range>
    </lidar>
    <visualize>true</visualize>
  </sensor>
</gazebo>
```

## 3. Bridge Configuration (ros_gz_bridge)

Create a `bridge.yaml` file in `config/` directory.

### Mapping Matrix
| Data Type | ROS 2 Type | Gazebo Type (Harmonic) |
| :--- | :--- | :--- |
| **Twist** | `geometry_msgs/msg/Twist` | `gz.msgs.Twist` |
| **LaserScan** | `sensor_msgs/msg/LaserScan` | `gz.msgs.LaserScan` |
| **Image** | `sensor_msgs/msg/Image` | `gz.msgs.Image` |
| **Odom** | `nav_msgs/msg/Odometry` | `gz.msgs.Odometry` |
| **Clock** | `rosgraph_msgs/msg/Clock` | `gz.msgs.Clock` |
| **TF** | `tf2_msgs/msg/TFMessage` | `gz.msgs.Pose_V` |

### YAML Example
```yaml
- ros_topic_name: "/cmd_vel"
  gz_topic_name: "/cmd_vel"
  ros_type_name: "geometry_msgs/msg/Twist"
  gz_type_name: "gz.msgs.Twist"
  direction: ROS_TO_GZ

- ros_topic_name: "/scan"
  gz_topic_name: "/scan"
  ros_type_name: "sensor_msgs/msg/LaserScan"
  gz_type_name: "gz.msgs.LaserScan"
  direction: GZ_TO_ROS

- ros_topic_name: "/clock"
  gz_topic_name: "/clock"
  ros_type_name: "rosgraph_msgs/msg/Clock"
  gz_type_name: "gz.msgs.Clock"
  direction: GZ_TO_ROS
```

## 4. Launch File Workflow

### Template
```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_my_bot = get_package_share_directory('my_bot_description')

    # KEY STEP: Set Resource Path
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.join(pkg_my_bot, '..')
    )

    # 1. Start Gazebo
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    # 2. Spawn Robot
    spawn = Node(
        package='ros_gz_sim', 
        executable='create',
        arguments=['-topic', 'robot_description'], 
        output='screen'
    )

    # 3. Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_my_bot, 'config', 'bridge.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )

    return LaunchDescription([gz_resource_path, gz_sim, spawn, bridge])
```
