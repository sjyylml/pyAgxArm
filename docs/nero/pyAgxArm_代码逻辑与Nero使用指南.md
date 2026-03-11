# pyAgxArm 代码逻辑梳理 & Nero 机械臂使用指南

## 一、项目概述

pyAgxArm 是松灵机器人 (Agilex Robotics) 开发的 Python SDK，用于通过 **CAN 总线** 控制旗下机械臂产品。

- 支持的机器人：`nero`（7自由度）、`piper`/`piper_h`/`piper_l`/`piper_x`（6自由度）
- 通信方式：SocketCAN（Linux）
- Python 版本：>= 3.6
- 依赖：`python-can>=3.3.4`, `typing-extensions>=3.7.4.3`

---

## 二、项目目录结构

```
pyAgxArm/
├── pyAgxArm/                          # 主包
│   ├── __init__.py                    # 导出: create_agx_arm_config, AgxArmFactory
│   ├── version.py                     # 版本号
│   ├── api/                           # 公共 API（工厂 + 配置）
│   │   ├── agx_arm_config.py          # create_agx_arm_config() 配置创建
│   │   ├── agx_arm_factory.py         # AgxArmFactory 工厂类
│   │   └── constants.py               # 关节限位/名称预设
│   ├── configs/                       # JSON 默认配置模板
│   │   ├── piper.json
│   │   └── nero.json
│   ├── demos/                         # 示例脚本
│   │   ├── nero/test1.py              # ★ Nero 使用示例
│   │   ├── piper/test1.py
│   │   └── ...
│   ├── scripts/                       # CAN 总线配置脚本 (Linux/Ubuntu)
│   ├── utiles/                        # 工具模块
│   │   ├── numeric_codec.py           # CAN 字节编解码
│   │   ├── fps.py                     # 帧率计算
│   │   ├── tf.py                      # 坐标变换（四元数/欧拉角/齐次矩阵）
│   │   └── vaildator.py              # 输入校验与限位钳制
│   └── protocols/can_protocol/        # CAN 协议层
│       ├── comms/                     # 通信层（CAN 总线收发）
│       ├── drivers/                   # 驱动层
│       │   ├── core/                  # 抽象基类和框架
│       │   ├── piper/                 # Piper 6-DOF 驱动
│       │   ├── nero/                  # ★ Nero 7-DOF 驱动
│       │   └── effector/             # 末端执行器驱动
│       │       ├── agx_gripper/       #   AGX 夹爪
│       │       └── revo2/             #   Revo2 灵巧手
│       └── msgs/                      # CAN 消息定义
│           ├── core/                  # 消息基类
│           ├── piper/                 # Piper 消息
│           ├── nero/                  # Nero 消息
│           └── effector/             # 末端执行器消息
├── docs/                              # 文档
├── setup.py
└── pyproject.toml
```

---

## 三、核心架构与代码逻辑

### 3.1 整体数据流

```
用户代码
  │
  ├── create_agx_arm_config()     ← 创建配置字典
  ├── AgxArmFactory.create_arm()  ← 工厂根据配置创建驱动实例
  ├── robot.connect()             ← 初始化 CAN 通信，启动收发线程
  │
  ├── [读取] robot.get_xxx()      ← 从解析器缓存中读取最新数据
  │      ↑
  │   读线程持续接收 CAN 帧 → Parser 表驱动解码 → 缓存到属性
  │
  └── [控制] robot.move_xxx()     ← 校验输入 → 编码 CAN 帧 → 发送
```

### 3.2 工厂模式

```python
# 入口：pyAgxArm/__init__.py 导出两个核心接口
from pyAgxArm import create_agx_arm_config, AgxArmFactory
```

- **`create_agx_arm_config(robot, comm, ...)`**：创建配置字典，自动填充关节限位等预设参数
- **`AgxArmFactory.create_arm(config)`**：内部维护一个注册表 `_registry`，根据 `(robot, comm, firmware_version)` 查找并实例化对应的驱动类

### 3.3 类继承体系

```
ArmDriverInterface (ABC)               # 接口定义
  └── ArmDriverAbstract                # 通用基类：connect, send, TCP offset, effector
        ├── PiperDriverDefault         # Piper 6-DOF 驱动（功能最全）
        │     ├── PiperHDriverDefault
        │     ├── PiperLDriverDefault
        │     └── PiperXDriverDefault
        └── NeroDriverDefault          # ★ Nero 7-DOF 驱动

ProtocolParserInterface (ABC)
  └── TableDriven                      # 表驱动 CAN 解析器
        └── PiperParser
              └── NeroParser           # ★ 继承 Piper，增加第 7 关节
```

### 3.4 表驱动 CAN 协议

核心机制在 `TableDriven` 类中：

| 方向 | 映射表 | 逻辑 |
|------|--------|------|
| **RX（接收）** | `CAN_ID → (属性名, 消息类, 解码函数)` | 收到 CAN 帧 → 查表 → 解码 → 缓存到 parser 属性 |
| **TX（发送）** | `msg.type_ → (CAN_ID, 编码函数)` | 构建消息对象 → 查表 → 编码 → 发送 CAN 帧 |

### 3.5 线程模型

`connect()` 后启动 3 个后台线程：

1. **读线程** (`_read_loop`)：持续调用 `comm.recv()`，触发 Parser 回调解码
2. **监控线程** (`_monitor_loop`)：每 50ms 检查通信健康状态
3. **FPS 线程** (`FPSManager`)：计算各消息的接收帧率

---

## 四、Nero 与 Piper 的区别

| 特性 | Piper (6-DOF) | Nero (7-DOF) |
|------|---------------|--------------|
| 关节数 | 6 | **7** |
| `move_j` 参数 | 6 元素列表 | **7 元素列表** |
| `move_l` (直线运动) | 支持 | 不支持 |
| `move_c` (圆弧运动) | 支持 | 不支持 |
| 运动模式 | P/J/L/C/MIT/JS | **仅 P/J** |
| 固件查询 | 支持 | 不支持 |
| 安装位置设置 | 支持 | 不支持 |
| 关节校准 | 支持 | 不支持 |
| 负载设置 | 支持 | 不支持 |

---

## 五、Nero 关节限位

| 关节 | 最小角度 (°) | 最大角度 (°) | 最小角度 (rad) | 最大角度 (rad) |
|------|-------------|-------------|---------------|---------------|
| Joint 1 | -155° | 155° | -2.705 | 2.705 |
| Joint 2 | -100° | 100° | -1.745 | 1.745 |
| Joint 3 | -158° | 158° | -2.758 | 2.758 |
| Joint 4 | -58° | 123° | -1.012 | 2.147 |
| Joint 5 | -158° | 158° | -2.758 | 2.758 |
| Joint 6 | -42° | 55° | -0.733 | 0.960 |
| Joint 7 | -90° | 90° | -1.571 | 1.571 |

---

## 六、Nero 使用指南

### 6.1 环境准备

#### 安装 SDK

```bash
cd pyAgxArm
pip install .
```

#### 配置 CAN 总线

确保 Linux 系统已加载 SocketCAN 驱动，然后启用 CAN 接口：

```bash
sudo ip link set can0 up type can bitrate 1000000
```

也可使用项目自带脚本（位于 `pyAgxArm/scripts/` 目录）。

### 6.2 基本使用流程

```python
import time
from pyAgxArm import create_agx_arm_config, AgxArmFactory

# 1. 创建配置
cfg = create_agx_arm_config(
    robot="nero",          # 机器人型号
    comm="can",            # 通信方式
    channel="can0",        # CAN 通道
    interface="socketcan"  # CAN 接口类型
)

# 2. 创建机械臂实例
robot = AgxArmFactory.create_arm(cfg)

# 3. 连接（启动通信线程）
robot.connect()

# 4. 设置为正常模式（单臂控制）
robot.set_normal_mode()

# 5. 使能所有关节电机
while not robot.enable():
    time.sleep(0.01)

# 6. 设置速度百分比
robot.set_speed_percent(50)  # 50% 速度

# --- 执行运动 ---

# 7. 完成后失能
while not robot.disable():
    time.sleep(0.01)
```

### 6.3 运动控制

#### Move J — 关节空间运动

将各关节移动到指定角度（弧度），需要传入 **7 个元素的列表**：

```python
# 设置运动模式为 J
robot.set_motion_mode(robot.OPTIONS.MOTION_MODE.J)

# 移动到指定关节角度（7个关节，单位：弧度）
robot.move_j([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
```

#### Move P — 笛卡尔空间运动

将末端法兰移动到指定位姿，参数为 `[x, y, z, roll, pitch, yaw]`：

```python
# 设置运动模式为 P
robot.set_motion_mode(robot.OPTIONS.MOTION_MODE.P)

# 移动到指定位姿（单位：米/弧度）
robot.move_p([0.3, 0.0, 0.4, 0.0, 0.0, 0.0])
```

**位姿约束：**
- `x, y, z`：位置，单位 m，精度 1e-6 m
- `roll, yaw`：范围 `[-π, π]`
- `pitch`：范围 `[-π/2, π/2]`
- 欧拉角约定：XYZ 外旋（等效 ZYX 内旋）

#### 等待运动完成

```python
def wait_motion_done(robot, timeout=5.0, poll_interval=0.1):
    """等待机械臂到达目标位置"""
    time.sleep(0.5)
    start = time.monotonic()
    while True:
        status = robot.get_arm_status()
        if status is not None and status.msg.motion_status == 0:
            return True  # 到达目标
        if time.monotonic() - start > timeout:
            return False  # 超时
        time.sleep(poll_interval)

# 使用
robot.move_j([0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
wait_motion_done(robot, timeout=5.0)
```

### 6.4 读取状态

```python
# 读取关节角度（7个关节，单位：弧度）
ja = robot.get_joint_angles()
if ja is not None:
    print("关节角度:", ja.msg)          # list[float], 长度7
    print("帧率:", ja.hz, "Hz")

# 读取末端位姿
fp = robot.get_flange_pose()
if fp is not None:
    x, y, z, roll, pitch, yaw = fp.msg
    print(f"位置: ({x}, {y}, {z}) m")
    print(f"姿态: ({roll}, {pitch}, {yaw}) rad")

# 读取机械臂状态
status = robot.get_arm_status()
if status is not None:
    print("控制模式:", status.msg.ctrl_mode)
    print("运动状态:", status.msg.motion_status)  # 0=已到位, 1=运动中
    print("臂状态:", status.msg.arm_status)        # 0=正常
    print("错误信息:", status.msg.err_status)

# 读取单个关节的驱动状态（关节索引 1~7）
ds = robot.get_driver_states(1)
if ds is not None:
    print("电压:", ds.msg.vol)
    print("电机温度:", ds.msg.motor_temp, "°C")
    print("使能状态:", ds.msg.foc_status.driver_enable_status)

# 读取单个关节的电机状态
ms = robot.get_motor_states(1)
if ms is not None:
    print("位置:", ms.msg.pos, "rad")
    print("速度:", ms.msg.motor_speed, "rad/s")
    print("扭矩:", ms.msg.torque, "N·m")

# 检查关节使能状态
enabled = robot.get_joint_enable_status(255)    # 255=检查所有关节
all_status = robot.get_joints_enable_status_list()  # 返回7个bool
```

### 6.5 紧急停止与复位

```python
# 阻尼急停（受控减速停止）
robot.electronic_emergency_stop()
time.sleep(1)

# 复位运动控制器
robot.reset()
```

### 6.6 末端执行器

Nero 支持两种末端执行器：

#### AGX 夹爪

```python
gripper = robot.init_effector(robot.OPTIONS.EFFECTOR.AGX_GRIPPER)

# 控制夹爪
gripper.move_gripper(width=0.05, force=1.5)  # 宽度 0~0.1m, 力 0~3.0N

# 读取状态
gs = gripper.get_gripper_status()
if gs is not None:
    print("开合宽度:", gs.msg.width, "m")

# 失能夹爪
gripper.disable_gripper()
```

#### Revo2 灵巧手

```python
hand = robot.init_effector(robot.OPTIONS.EFFECTOR.REVO2)

# 位置控制（每个手指 0~100）
hand.position_ctrl(
    thumb_tip=50,
    thumb_base=50,
    index_finger=50,
    middle_finger=50,
    ring_finger=50,
    pinky_finger=50
)

# 读取手指位置
pos = hand.get_finger_pos()
```

> **注意：** 末端执行器只能初始化一次，不能切换类型。

### 6.7 TCP 偏移（工具中心点）

```python
# 设置 TCP 偏移（相对于法兰坐标系）
robot.set_tcp_offset([0, 0, 0.1, 0, 0, 0])  # Z 方向偏移 0.1m

time.sleep(0.1)

# 获取 TCP 位姿
tcp_pose = robot.get_tcp_pose()
print(tcp_pose)

# 法兰位姿 → TCP 位姿
flange_pose = robot.get_flange_pose()
if flange_pose is not None:
    tcp_pose = robot.get_flange2tcp_pose(flange_pose.msg)

# TCP 位姿 → 法兰位姿（用于将 TCP 目标转为 move_p 参数）
flange_target = robot.get_tcp2flange_pose([0.3, 0.0, 0.5, 0.0, 0.0, 0.0])
robot.move_p(flange_target)
```

### 6.8 主从臂模式（遥操作）

```python
# ---- 主臂端 ----
master = AgxArmFactory.create_arm(
    create_agx_arm_config(robot="nero", comm="can", channel="can0")
)
master.connect()
master.set_master_mode()  # 设为主臂（零力拖动模式）

# ---- 从臂端 ----
slave = AgxArmFactory.create_arm(
    create_agx_arm_config(robot="nero", comm="can", channel="can1")
)
slave.connect()
slave.set_slave_mode()    # 设为从臂（跟随模式）

# ---- 恢复单臂模式 ----
robot.set_normal_mode()
```

---

## 七、完整使用示例

```python
import time
from pyAgxArm import create_agx_arm_config, AgxArmFactory


def wait_motion_done(robot, timeout=5.0):
    time.sleep(0.5)
    start = time.monotonic()
    while True:
        status = robot.get_arm_status()
        if status is not None and status.msg.motion_status == 0:
            print("到达目标位置")
            return True
        if time.monotonic() - start > timeout:
            print(f"运动超时 ({timeout}s)")
            return False
        time.sleep(0.1)


# ========== 初始化 ==========
cfg = create_agx_arm_config(robot="nero", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()

# ========== 使能 ==========
robot.set_normal_mode()
while not robot.enable():
    time.sleep(0.01)
print("所有关节已使能")

robot.set_speed_percent(50)

# ========== 运动 ==========
# 关节空间运动：回零位
robot.move_j([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
wait_motion_done(robot)

# 笛卡尔空间运动
robot.move_p([0.3, 0.0, 0.4, 0.0, 0.0, 0.0])
wait_motion_done(robot)

# ========== 读取数据 ==========
ja = robot.get_joint_angles()
if ja is not None:
    print(f"关节角度: {ja.msg}")

fp = robot.get_flange_pose()
if fp is not None:
    print(f"末端位姿: {fp.msg}")

# ========== 失能 ==========
while not robot.disable():
    time.sleep(0.01)
print("所有关节已失能")
```

---

## 八、常见问题

### Q: `robot.enable()` 一直返回 False？
确保：
1. CAN 总线已正确启用 (`ip link show can0` 查看状态)
2. 机械臂已上电
3. 没有处于急停状态（需先 `robot.reset()`）

### Q: `get_joint_angles()` 返回 None？
连接后需要等待数据帧到达，通常 `connect()` 后等待约 0.5s 再读取。

### Q: 运动模式 `move_l` / `move_c` 报错？
Nero 不支持直线运动 (`move_l`) 和圆弧运动 (`move_c`)，只支持 `move_p` 和 `move_j`。

### Q: 如何检查通信是否正常？
```python
print(robot.is_ok())     # 通信健康状态
print(robot.get_fps())   # 接收帧率，正常应 > 0
```

---

## 九、API 速查表

| 方法 | 说明 | Nero 支持 |
|------|------|----------|
| `connect()` | 连接机械臂 | Yes |
| `enable(joint_index=255)` | 使能关节 (1-7, 255=全部) | Yes |
| `disable(joint_index=255)` | 失能关节 | Yes |
| `reset()` | 复位控制器 | Yes |
| `electronic_emergency_stop()` | 阻尼急停 | Yes |
| `set_speed_percent(0~100)` | 设置速度百分比 | Yes |
| `set_motion_mode('p'/'j')` | 设置运动模式 | Yes |
| `move_p([x,y,z,r,p,y])` | 笛卡尔运动 | Yes |
| `move_j([j1..j7])` | 关节运动 (7个关节) | Yes |
| `get_joint_angles()` | 获取关节角度 | Yes |
| `get_flange_pose()` | 获取法兰位姿 | Yes |
| `get_arm_status()` | 获取状态 | Yes |
| `get_driver_states(1~7)` | 驱动器状态 | Yes |
| `get_motor_states(1~7)` | 电机状态 | Yes |
| `set_tcp_offset(pose)` | 设置 TCP 偏移 | Yes |
| `get_tcp_pose()` | 获取 TCP 位姿 | Yes |
| `set_normal_mode()` | 普通模式 | Yes |
| `set_master_mode()` | 主臂模式 | Yes |
| `set_slave_mode()` | 从臂模式 | Yes |
| `init_effector(type)` | 初始化末端执行器 | Yes |
| `move_l()` | 直线运动 | **No** |
| `move_c()` | 圆弧运动 | **No** |
| `calibrate_joint()` | 关节校准 | **No** |
| `set_installation_pos()` | 安装位置 | **No** |
| `get_firmware()` | 固件信息 | **No** |
