# SetModeNode 飞行模式文档

## 概述

`SetModeNode` 是UAV行为树中的一个关键节点，用于直接向PX4飞控发送飞行模式切换命令。该节点采用PX4直接命令模式，通过标准MAVLink协议与飞控通信，实现实时、可靠的模式切换功能。

## 设计原理

### 通信架构
```
BehaviorTree → SetModeNode → /fmu/in/vehicle_command → PX4 飞控
```

- **直接通信**：绕过ROS服务层，直接与PX4通信
- **标准协议**：使用MAVLink `DO_SET_MODE` 命令
- **实时性**：单向发布，无阻塞等待
- **兼容性**：支持所有PX4版本

### 命令格式
```cpp
command_msg.command = VEHICLE_CMD_DO_SET_MODE;
command_msg.param1 = base_mode;      // MAV_MODE
command_msg.param2 = custom_main_mode; // PX4主模式
command_msg.param3 = custom_sub_mode;  // PX4子模式
```

## 支持的飞行模式

### 手动控制模式

#### 1. MANUAL - 完全手动模式
- **用途**：完全手动控制，所有控制轴由遥控器直接控制
- **特点**：无任何飞控辅助，需要丰富的飞行经验
- **适用场景**：专业飞手、极限飞行、紧急情况
- **编码**：`base_mode=1, custom_main_mode=1`

#### 2. ACRO - 特技模式
- **用途**：角速度控制模式，用于特技飞行
- **特点**：遥控器控制角速度，飞控稳定角速度
- **适用场景**：特技表演、竞速飞行
- **编码**：`base_mode=1, custom_main_mode=2`

#### 3. STABILIZED - 自稳模式
- **用途**：姿态稳定模式
- **特点**：遥控器控制姿态角度，飞控自动保持水平
- **适用场景**：初学者训练、温和飞行
- **编码**：`base_mode=1, custom_main_mode=7`

#### 4. RATTITUDE - 混合姿态模式
- **用途**：结合ACRO和STABILIZED的混合模式
- **特点**：小幅度操作为姿态模式，大幅度操作为角速度模式
- **适用场景**：过渡训练、灵活操控
- **编码**：`base_mode=1, custom_main_mode=8`

### 辅助控制模式

#### 5. ALTCTL - 高度控制模式
- **用途**：高度保持+姿态控制
- **特点**：飞控自动保持高度，手动控制水平姿态
- **适用场景**：航拍、定高飞行
- **编码**：`base_mode=1, custom_main_mode=3`

#### 6. POSCTL - 位置控制模式
- **用途**：三维位置保持模式
- **特点**：GPS定位，悬停保持，遥控器控制移动
- **适用场景**：精确悬停、慢速机动
- **编码**：`base_mode=1, custom_main_mode=4`

### 自动飞行模式

#### 7. AUTO_MISSION - 自动任务模式
- **用途**：执行预设的飞行任务
- **特点**：按照waypoint自动飞行
- **适用场景**：测绘、巡检、自动配送
- **编码**：`base_mode=1, custom_main_mode=5, custom_sub_mode=1`

#### 8. AUTO_LOITER - 自动悬停模式
- **用途**：在当前位置自动悬停
- **特点**：GPS保持位置，自动抗风
- **适用场景**：临时悬停、等待指令
- **编码**：`base_mode=1, custom_main_mode=5, custom_sub_mode=3`

#### 9. AUTO_RTL - 自动返航模式
- **用途**：自动返回起飞点并降落
- **特点**：安全返航、自动降落
- **适用场景**：紧急返航、任务结束
- **编码**：`base_mode=1, custom_main_mode=5, custom_sub_mode=4`

#### 10. AUTO_TAKEOFF - 自动起飞模式
- **用途**：自动起飞到指定高度
- **特点**：垂直起飞、高度控制
- **适用场景**：任务开始、标准起飞
- **编码**：`base_mode=1, custom_main_mode=5, custom_sub_mode=2`

#### 11. AUTO_LAND - 自动降落模式
- **用途**：自动降落到当前位置下方
- **特点**：垂直降落、软着陆
- **适用场景**：任务结束、安全降落
- **编码**：`base_mode=1, custom_main_mode=5, custom_sub_mode=5`

#### 12. AUTO_FOLLOW_TARGET - 目标跟随模式
- **用途**：自动跟随指定目标
- **特点**：视觉跟踪、动态跟随
- **适用场景**：目标跟踪、智能跟拍
- **编码**：`base_mode=1, custom_main_mode=5, custom_sub_mode=6`

#### 13. AUTO_PRECLAND - 精确降落模式
- **用途**：使用视觉标记精确降落
- **特点**：视觉导航、厘米级精度
- **适用场景**：精确投放、定点降落
- **编码**：`base_mode=1, custom_main_mode=5, custom_sub_mode=7`

### 外部控制模式

#### 14. OFFBOARD - 外部控制模式
- **用途**：接受外部计算机的位置/速度指令
- **特点**：程序化控制、高精度机动
- **适用场景**：自主导航、复杂任务执行
- **编码**：`base_mode=1, custom_main_mode=6`

## 使用方法

### 在行为树XML中使用

```xml
<!-- 设置为位置控制模式 -->
<SetMode mode="POSCTL" />

<!-- 设置为自动任务模式 -->
<SetMode mode="AUTO_MISSION" />

<!-- 设置为外部控制模式 -->
<SetMode mode="OFFBOARD" />
```

### 在C++代码中使用

```cpp
// 创建节点实例
auto set_mode_node = std::make_shared<SetModeNode>("set_mode", config, node_handle);

// 设置输入端口
set_mode_node->setInput("mode", "POSCTL");

// 执行节点
auto status = set_mode_node->tick();
```

### 动态模式设置

```xml
<!-- 从blackboard获取模式 -->
<SetMode mode="{target_flight_mode}" />

<!-- 在序列中使用 -->
<Sequence>
    <SetBlackboard output_key="target_flight_mode" value="AUTO_TAKEOFF" />
    <SetMode mode="{target_flight_mode}" />
    <WaitSeconds time="5" />
    <SetBlackboard output_key="target_flight_mode" value="AUTO_MISSION" />
    <SetMode mode="{target_flight_mode}" />
</Sequence>
```

## 飞行模式选择指南

### 根据任务阶段选择

| 阶段 | 推荐模式 | 备选模式 |
|------|----------|----------|
| 起飞 | AUTO_TAKEOFF | POSCTL |
| 航路飞行 | AUTO_MISSION | OFFBOARD |
| 悬停等待 | AUTO_LOITER | POSCTL |
| 目标搜索 | OFFBOARD | POSCTL |
| 精确操作 | OFFBOARD | POSCTL |
| 紧急返航 | AUTO_RTL | POSCTL |
| 精确降落 | AUTO_PRECLAND | AUTO_LAND |

### 根据控制需求选择

| 控制需求 | 推荐模式 | 说明 |
|----------|----------|------|
| 完全自主 | AUTO_MISSION | 预设航点飞行 |
| 程序控制 | OFFBOARD | 外部算法控制 |
| 手动控制 | POSCTL | GPS辅助手动 |
| 定点悬停 | AUTO_LOITER | 自动保持位置 |
| 紧急操作 | MANUAL | 完全手动控制 |

### 根据环境条件选择

| 环境条件 | 推荐模式 | 注意事项 |
|----------|----------|----------|
| GPS良好 | POSCTL, AUTO_* | 可使用所有GPS模式 |
| GPS较差 | ALTCTL, STABILIZED | 避免位置控制模式 |
| 室内环境 | MANUAL, ACRO | 无GPS环境 |
| 复杂环境 | OFFBOARD | 需要精确控制 |

## 安全注意事项

### 模式切换前置条件

1. **传感器状态**：确保相关传感器正常工作
2. **GPS状态**：GPS模式需要3D定位
3. **电池电量**：低电量时某些模式可能被拒绝
4. **飞行状态**：某些模式只能在特定状态下切换

### 失效保护

- **模式切换失败**：PX4会保持当前模式
- **传感器失效**：自动降级到安全模式
- **通信中断**：触发失效保护程序
- **低电量**：强制返航或降落

### 最佳实践

1. **分阶段切换**：避免直接从手动模式跳转到复杂自动模式
2. **状态监控**：通过`/fmu/out/vehicle_status`监控实际模式
3. **备用方案**：为每个模式准备备用切换方案
4. **测试验证**：在仿真环境中充分测试模式切换逻辑

## 故障排除

### 常见问题

#### 模式切换被拒绝
- **原因**：前置条件不满足
- **解决**：检查传感器状态、GPS状态、电池电量

#### 切换延迟
- **原因**：飞控需要时间验证切换条件
- **解决**：等待状态稳定后再进行下一步操作

#### 模式不生效
- **原因**：字符串拼写错误或不支持的模式
- **解决**：检查模式名称拼写，参考支持列表

### 调试方法

```bash
# 监控飞控状态
ros2 topic echo /fmu/out/vehicle_status

# 查看命令发送情况
ros2 topic echo /fmu/in/vehicle_command

# 检查行为树日志
ros2 topic echo /behavior_tree_log
```

## 示例：完整飞行流程

```xml
<BehaviorTree ID="CompleteFlightMission">
    <Sequence name="flight_sequence">
        <!-- 解锁 -->
        <Arm arm="true" />
        
        <!-- 起飞 -->
        <SetMode mode="AUTO_TAKEOFF" />
        <WaitForHeight target_height="10.0" />
        
        <!-- 执行任务 -->
        <SetMode mode="AUTO_MISSION" />
        <WaitForMissionComplete />
        
        <!-- 悬停等待 -->
        <SetMode mode="AUTO_LOITER" />
        <WaitSeconds time="5" />
        
        <!-- 返航降落 -->
        <SetMode mode="AUTO_RTL" />
        <WaitForLanded />
        
        <!-- 上锁 -->
        <Arm arm="false" />
    </Sequence>
</BehaviorTree>
```

---

**版本信息**
- 文档版本：v2.0
- 更新日期：2025年8月2日
- 作者：王为鸣
- 适用于：PX4 v1.12+, ROS2 Humble
