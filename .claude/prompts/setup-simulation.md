---
description: Setup a new ROS 2 simulation environment with Gazebo Sim
---

# Setup Simulation Workflow

This workflow guides you through setting up a Gazebo Sim simulation for a robot.

## 1. Check Compatibility
- Verify ROS 2 version (`printenv ROS_DISTRO`).
- Use `view_file .agent/rules/simulation-standards.md` to check compatible Gazebo version and bridge package.

## 2. Prepare Assets (URDF)
- Ensure your robot description is in `src/<robot_description>`.
- Open your URDF/Xacro file.
- Add Gazebo Sim System Plugins using the `simulation-expert` skill reference.
  - Diff Drive: `gz-sim-diff-drive-system` (Jazzy) or `ignition-gazebo-diff-drive-system` (Humble).
  - Sensors: Add `<sensor>` tags and `gz::sim::systems::Sensors`.

## 3. Configure Bridge
- Create directory `config` in your package.
- Create file `config/bridge.yaml`.
- Define topics to bridge (cmd_vel, odom, scan, image, clock).
- Use `view_file .agent/skills/simulation-expert/SKILL.md` for the mapping matrix.

## 4. Create Launch File
- Create `launch/sim.launch.py`.
- Apply the "Launch File Workflow" from `simulation-expert` skill.
  - Set `GZ_SIM_RESOURCE_PATH`.
  - Include `gz_sim.launch.py`.
  - Spawn robot using `create` node.
  - Run `parameter_bridge` with your YAML.

## 5. Build and Run
// turbo
- Run `colcon build --symlink-install`
- Source workspace: `source install/setup.bash`
- Launch: `ros2 launch <package> sim.launch.py`
