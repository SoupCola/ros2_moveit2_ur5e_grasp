# ROS 2 Skills - 快速参考

## 技能调用方式

### 通过自然语言自动触发
Claude 会根据你的请求自动选择合适的技能：

| 你的请求 | 自动使用的技能 |
|----------|----------------|
| "创建一个 ROS 2 包" | ros2_package_scaffolder |
| "构建项目" | ros2_build_master |
| "修复构建错误" | ros2_build_resolver |
| "审查这段代码" | code-review |
| "调试 TF2/Topic 问题" | ros-expert |
| "设计系统架构" | system-architect |
| "解决 Git 冲突" | git-expert |
| "配置生命周期节点" | lifecycle_operator |
| "设置仿真环境" | simulation-expert |
| "编写单元测试" | tdd-ros2 |

### 通过工作流提示手动触发
你可以引用 `.claude/prompts/` 目录下的工作流文件：

| 文件 | 功能 | 触发方式 |
|------|------|----------|
| `ci-local-pipeline.md` | 运行完整 CI/CD 流程 | "运行 ci-local-pipeline" |
| `clean-init.md` | 清理并重置工作空间 | "运行 clean-init" |
| `install-ros2.md` | 安装 ROS 2 Humble | "运行 install-ros2" |
| `setup-simulation.md` | 设置仿真环境 | "运行 setup-simulation" |
| `universal-request.md` | 标准请求处理流程 | "运行 universal-request" |

## 自动应用的规则

以下规则会自动应用于你的代码：

| 规则文件 | 应用范围 | 主要内容 |
|----------|----------|----------|
| `ros2-architecture.md` | 全局 | 构建类型分离、组件化设计 |
| `ros2-cpp-style.md` | C++ 文件 | OOP、命名规范、智能指针 |
| `ros2-python-style.md` | Python 文件 | PEP 8、节点继承、类型提示 |
| `simulation-standards.md` | 仿真相关 | 仿真环境配置标准 |
| `technical-standards.md` | 全局 | KISS、DRY、YAGNI 原则 |

## MCP 工具

可用的 Model Context Protocol 工具：

| 工具 | 功能 | 使用方法 |
|------|------|----------|
| ros2-docs | 访问 ROS 2 Humble 文档 | Claude 可查询 `/opt/ros/humble/share/doc` |
| project-context | 获取 ROS 2 图信息 | 获取当前节点、主题、服务列表 |

## 常用命令参考

### 构建
```bash
# 完整构建（使用符号链接，开发时推荐）
colcon build --symlink-install

# 选择性构建（只构建指定包）
colcon build --packages-select <package_name> --symlink-install

# Debug 构建（包含调试符号）
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug --symlink-install
```

### 测试
```bash
# 运行 linter 测试
colcon test --packages-select <package_name> --ctest-args -R "flake8|cpplint|clang_format|pep257"

# 运行单元测试
colcon test --packages-select <package_name> --ctest-args -E "lint"

# 查看测试结果
colcon test-result --verbose
```

### 清理
```bash
# 清理构建产物
rm -rf build/ install/ log/
```

## 技能详情

### ros2_package_scaffolder
自动创建标准 ROS 2 包结构：
- Ament CMake (C++)
- Ament Python
- Interface (msg/srv/action)

### ros2_build_master
智能构建管理：
- 支持符号链接安装
- 选择性构建
- Debug 模式
- 错误诊断

### ros2_build_resolver
修复构建错误：
- CMake 配置问题
- 链接错误
- 依赖缺失

### ros-expert
ROS 2 运行时调试：
- TF2 树问题
- Topic 通信
- Service 调用
- QoS 策略

### system_architect
系统架构设计：
- 节点图设计
- QoS 配置
- 命名空间规划
- 性能优化

### code_review
六维代码审查：
1. 架构设计
2. 安全性
3. 性能
4. 可维护性
5. 测试覆盖
6. 文档完整性

### tdd_ros2
测试驱动开发：
- GTest (C++)
- PyTest (Python)
- 测试结构与最佳实践
