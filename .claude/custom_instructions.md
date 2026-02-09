# ROS 2 开发环境 - Claude 自定义指令

## 项目概述
这是一个基于 UR5e 机械臂的 ROS 2 抓取项目，使用 MoveIt2 进行运动规划。

## 可用资源

### 技能 (Skills)
位于 `.claude/skills/` 目录，包含以下专门技能：

| 技能 | 功能 | 使用场景 |
|------|------|----------|
| **ros2_package_scaffolder** | 创建标准 ROS 2 包 | 用户要求创建新包时 |
| **ros2_build_master** | 智能构建管理 | 运行工作空间构建时 |
| **lifecycle_operator** | 管理生命周期节点 | 配置或测试硬件驱动时 |
| **ros-expert** | 调试 TF2, Topics, Services, QoS | 分析运行时/通信问题时 |
| **tdd-ros2** | 测试驱动开发 (GTest/Pytest) | 编写单元测试或新功能时 |
| **ros2-build-resolver** | 修复 CMake, 链接, 依赖错误 | `colcon build` 失败时 |
| **system-architect** | 设计节点图、QoS、命名空间 | 设计系统架构时 |
| **code-review** | 6方面代码审查 | 用户要求代码/PR 审查时 |
| **git-expert** | 解决合并冲突、分支管理 | 处理 Git 操作时 |
| **simulation-expert** | 仿真相关问题 | 处理 Gazebo/仿真问题时 |

### 规则 (Rules)
位于 `.claude/rules/` 目录，始终激活的编码标准：

| 规则 | 触发条件 | 描述 |
|------|----------|------|
| **ros2-architecture** | 全局 | 分离构建类型 (C++/Python/接口)，组件化设计 |
| **ros2-cpp-style** | `*.cpp`, `*.hpp` | 严格 OOP，命名规范，禁止 `new/delete` |
| **ros2-python-style** | `*.py` | PEP 8，节点必须继承 `rclpy.node.Node` |
| **technical-standards** | 全局 | KISS, DRY, YAGNI 原则 |
| **simulation-standards** | 仿真相关 | 仿真环境配置标准 |

### 工作流 (Workflows)
位于 `.claude/prompts/` 目录，可通过斜杠命令触发：

| 命令 | 功能 |
|------|------|
| `/ci-local-pipeline` | 运行本地 CI/CD: rosdep → lint → build → test |
| `/clean-init` | 清理构建产物，重置工作空间 |
| `/install-ros2` | 在 Ubuntu 22.04 上安装 ROS 2 Humble |
| `/setup-simulation` | 设置仿真环境 |
| `/universal-request` | 标准请求处理流程 |

## 使用指南

### 1. 自然交互（推荐）
用户可以用自然语言描述目标，你应该自动选择合适的技能：
- *"创建一个 Python 包来控制电机"* → 使用 `ros2_package_scaffolder`
- *"代码看起来有问题，帮我审查一下"* → 使用 `code-review`
- *"构建失败了，请修复"* → 使用 `ros2-build-resolver`

### 2. 自动规则应用
编写代码时自动遵循：
- C++: 使用 2 空格缩进，snake_case 方法名，成员变量加 `_` 后缀
- Python: 遵循 PEP 8，节点继承 `rclpy.node.Node`
- 架构: 组件化设计，使用 `rclcpp_components`

### 3. MCP 服务器
可用的 MCP 工具：
- **ros2-docs**: 访问 `/opt/ros/humble/share/doc` 下的 ROS 2 文档
- **project-context**: 获取当前 ROS 2 图信息（节点、主题、服务）

## 项目结构
```
ros2_moveit2_ur5e_grasp/
├── src/
│   ├── ur5e_grasp/           # 主要抓取包
│   ├── vision/               # 视觉检测包
│   └── ...
├── launch/                   # 启动文件
├── config/                   # 配置文件
├── .claude/                  # Claude 配置和资源
│   ├── skills/               # AI 技能定义
│   ├── rules/                # 编码规则
│   ├── prompts/              # 工作流提示
│   ├── tools/                # 辅助脚本
│   └── mcp_servers.json      # MCP 配置
└── ros2-skills/              # 原始技能仓库（参考）
```

## 常见任务处理

### 创建新包
1. 使用 `ros2_package_scaffolder` 技能
2. 根据语言选择合适的构建类型（ament_cmake/ament_python）
3. 自动配置 linter 和依赖

### 构建失败
1. 使用 `ros2-build-resolver` 技能
2. 检查 `package.xml` 依赖
3. 验证 `CMakeLists.txt` 语法
4. 查看构建日志

### 代码审查
1. 使用 `code-review` 技能
2. 检查 6 个方面：架构、安全、性能、可维护性、测试、文档

### 运行测试
1. 使用 `tdd-ros2` 技能
2. C++: 使用 GTest
3. Python: 使用 PyTest

## 重要注意事项
1. 所有 ROS 2 命令前必须先 source 环境：`source /opt/ros/humble/setup.bash`
2. 使用 `colcon build --symlink-install` 进行开发构建
3. Python 节点必须使用 `entry_points` 而非直接调用 `.py` 文件
4. C++ 节点应使用组件模式（继承 `rclcpp::Node`）
