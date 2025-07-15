# 机械臂坐标控制 - 快速使用指南

## 功能概述

机械臂硬件节点现在支持通过标准的ROS2 `PoseStamped` 消息类型分别控制左臂和右臂的坐标位置和姿态。

## 快速开始

### 1. 启动机械臂硬件节点

```bash
ros2 run arm_ik agi_hw_ros2.py
```

### 2. 控制左臂

```bash
# 发送左臂控制指令
ros2 topic pub /agi_hw/left_arm_target_pose geometry_msgs/PoseStamped "{
  header: {frame_id: 'base_link'},
  pose: {
    position: {x: 0.8, y: 0.2, z: 1.1},
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  }
}"
```

### 3. 控制右臂

```bash
# 发送右臂控制指令
ros2 topic pub /agi_hw/right_arm_target_pose geometry_msgs/PoseStamped "{
  header: {frame_id: 'base_link'},
  pose: {
    position: {x: 0.8, y: -0.2, z: 1.1},
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  }
}"
```

### 4. 使用测试脚本

```bash
# 运行自动测试脚本
python3 scripts/test_arm_control.py
```

## 话题信息

| 话题名称 | 消息类型 | 功能 |
|---------|---------|------|
| `/agi_hw/left_arm_target_pose` | `geometry_msgs/PoseStamped` | 控制左臂位置和姿态 |
| `/agi_hw/right_arm_target_pose` | `geometry_msgs/PoseStamped` | 控制右臂位置和姿态 |

## 常用控制示例

### 基本位置控制

```bash
# 左臂移动到前方
ros2 topic pub /agi_hw/left_arm_target_pose geometry_msgs/PoseStamped "{
  header: {frame_id: 'base_link'},
  pose: {
    position: {x: 0.8, y: 0.2, z: 1.1},
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  }
}"

# 右臂移动到右侧
ros2 topic pub /agi_hw/right_arm_target_pose geometry_msgs/PoseStamped "{
  header: {frame_id: 'base_link'},
  pose: {
    position: {x: 0.8, y: -0.2, z: 1.1},
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  }
}"
```

### 姿态控制

```bash
# 左臂垂直向下
ros2 topic pub /agi_hw/left_arm_target_pose geometry_msgs/PoseStamped "{
  header: {frame_id: 'base_link'},
  pose: {
    position: {x: 0.8, y: 0.2, z: 0.5},
    orientation: {x: 0.707, y: 0.0, z: 0.0, w: 0.707}
  }
}"

# 右臂水平旋转90度
ros2 topic pub /agi_hw/right_arm_target_pose geometry_msgs/PoseStamped "{
  header: {frame_id: 'base_link'},
  pose: {
    position: {x: 0.8, y: -0.2, z: 0.8},
    orientation: {x: 0.0, y: 0.0, z: 0.707, w: 0.707}
  }
}"
```

## 注意事项

1. **坐标系**: 使用 `base_link` 坐标系
2. **单位**: 位置单位为米
3. **四元数**: 姿态使用四元数表示，模长应为1
4. **安全**: 确保目标位置在机械臂工作空间内
5. **独立控制**: 左臂和右臂可以独立控制

## 故障排除

- **机械臂不响应**: 检查机械臂控制器状态和网络连接
- **运动失败**: 检查目标位置是否在工作空间内
- **姿态异常**: 确认四元数模长接近1

## 更多信息

详细使用说明请参考：`docs/arm_control_usage.md` 