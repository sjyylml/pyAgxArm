# Nero 力控模块开发文档

> 模块路径: `pyAgxArm/protocols/can_protocol/drivers/nero/force_ctrl/`

---

## 1. 背景与目标

pyAgxArm 原生的 Nero 驱动仅提供 `move_p` / `move_j` 两种位置级控制——用户给目标位姿或关节角，主控固件内部完成轨迹规划后执行。这种模式无法满足柔顺装配、接触力控等场景的需求。

本模块在 **不改变 pyAgxArm 原有架构** 的前提下，参考 i2rt SDK 的实时控制架构，新增了：

1. Nero 驱动层 MIT 阻抗控制 API（`move_mit` / `move_mit_batch`）
2. 基于 MuJoCo 逆动力学的实时重力补偿
3. 200 Hz 后台 MIT 控制循环
4. 高层 `NeroForceController` 接口（零重力拖动、位置 PD、全参数 MIT）

---

## 2. 文件结构

```
pyAgxArm/
├── protocols/can_protocol/drivers/nero/
│   ├── default/
│   │   └── driver.py                  # [已修改] 新增 move_mit / move_mit_batch
│   └── force_ctrl/                    # [新增] 力控模块
│       ├── __init__.py                # 导出 NeroForceController
│       ├── data_types.py              # 数据结构: JointCommands, JointState
│       ├── gravity_comp.py            # MuJoCo 重力补偿器
│       ├── mit_loop.py                # 200Hz 实时控制循环
│       └── controller.py             # 高层接口 NeroForceController
├── robot_models/nero/                 # [新增] 机器人模型
│   ├── nero_description.urdf          # 完整 URDF（含 STL mesh 引用，重力补偿时自动剥离 visual/collision）
│   └── nero_description/meshes/*.STL  # 8 个 STL mesh 文件（可视化用）
└── demos/nero/
    └── force_ctrl_demo.py             # [新增] 力控演示脚本
```

---

## 3. 架构总览

```
┌──────────────────────────────────────────────────────────┐
│                   用户代码                                │
│  ctrl = NeroForceController(channel="can0")              │
│  ctrl.start()                                            │
│  ctrl.zero_gravity_mode()                                │
│  ctrl.command_joint_pos(target_q, kp=..., kd=...)        │
│  ctrl.command_joint_state(pos, vel, kp, kd, torques)     │
│  state = ctrl.get_joint_state()                          │
│  ctrl.stop()                                             │
└──────────────────────┬───────────────────────────────────┘
                       │ update_commands()  (线程安全)
                       ▼
┌──────────────────────────────────────────────────────────┐
│            NeroMITLoop  (后台线程, 200 Hz)                │
│                                                          │
│  每个周期 (5ms):                                          │
│  1. 从 parser 缓存读取关节角度 q                          │
│  2. 安全检查 (限位 / 跳变 / 心跳)                         │
│  3. MuJoCo 计算重力补偿力矩 τ_g = mj_inverse(q,0,0)      │
│  4. 合并: τ_total = τ_user + τ_g × gravity_factor        │
│  5. clamp τ_total 到 [-torque_limit, torque_limit]       │
│  6. 对 7 个关节各发一帧 MIT CAN 命令                      │
│     (pos, vel, kp, kd, τ_total)                          │
│  7. 更新 joint_state 供用户读取                           │
└──────────────────────┬───────────────────────────────────┘
                       │ move_mit_batch()
                       ▼
┌──────────────────────────────────────────────────────────┐
│         Nero Driver (pyAgxArm 原生)                       │
│                                                          │
│  move_mit_batch()                                        │
│    → _validate_and_quantize_mit_params()  (校验+量化)     │
│    → _parser._make_joint_mit_ctrl_msg()   (构建 CAN 消息) │
│    → _send_msg()                          (发送 CAN 帧)  │
│                                                          │
│  CAN TX:  0x15A~0x160 (7 个关节的 MIT 控制帧)             │
│  CAN TX:  0x151        (模式控制: MIT mode = 0xAD)        │
└──────────────────────┬───────────────────────────────────┘
                       │ CAN Bus
                       ▼
┌──────────────────────────────────────────────────────────┐
│         pyAgxArm CAN 读线程 (已有)                        │
│                                                          │
│  持续接收反馈帧 → parser 解码 → 缓存到属性                 │
│                                                          │
│  CAN RX:  0x2A5~0x2A9 (关节角度)                         │
│  CAN RX:  0x251~0x257 (电机高速反馈: 位置/速度/电流/力矩)  │
│  CAN RX:  0x261~0x267 (驱动器低速反馈: 电压/温度/状态)     │
│  CAN RX:  0x2A1        (臂状态/错误)                      │
└──────────────────────────────────────────────────────────┘
```

**线程模型（共 3 个线程）：**

| 线程 | 来源 | 频率 | 职责 |
|------|------|------|------|
| pyAgxArm 读线程 | `robot.connect()` 创建 | 尽快读 | 接收 CAN 反馈帧，解码缓存到 parser 属性 |
| MIT 控制循环 | `NeroMITLoop.start()` 创建 | 200 Hz | 重力补偿 + 合并指令 + 发送 MIT CAN 帧 |
| 用户线程 | 用户代码 | 用户决定 | 调用 `command_*` 设置目标，调用 `get_*` 读取状态 |

**与 i2rt 的关键差异：**

pyAgxArm 使用**广播式** CAN 通信（发送和接收在不同 CAN ID 上独立进行），而 i2rt 使用**请求-响应式**（发一帧命令、等一帧回复）。因此本模块的控制循环**不需要**在发送后等待 CAN 响应——反馈数据由 pyAgxArm 已有的读线程持续更新到 parser 缓存中。

---

## 4. 各模块详解

### 4.1 data_types.py — 数据结构

**文件**: `force_ctrl/data_types.py`

```python
@dataclass
class JointCommands:
    pos: np.ndarray       # (7,) 目标关节角度 (rad)
    vel: np.ndarray       # (7,) 目标关节速度 (rad/s)
    kp:  np.ndarray       # (7,) 比例增益
    kd:  np.ndarray       # (7,) 微分增益
    torques: np.ndarray   # (7,) 用户前馈力矩 (N·m)，不含重力补偿

@dataclass
class JointState:
    pos: np.ndarray       # (7,) 当前关节角度 (rad)
    vel: np.ndarray       # (7,) 当前关节速度 (rad/s)
    torque: np.ndarray    # (7,) 当前关节力矩 (N·m)
    timestamp: float      # 时间戳 (s)
```

`JointCommands.zeros(7)` 返回全零的命令结构，用于初始化。

---

### 4.2 gravity_comp.py — 重力补偿器

**文件**: `force_ctrl/gravity_comp.py`
**类**: `NeroGravityCompensator`

**原理：**

调用 MuJoCo 的 `mj_inverse` 逆动力学函数。设定 `qpos=q, qvel=0, qacc=0`，得到的 `qfrc_inverse` 就是在当前关节构型下、零速度零加速度时需要施加的关节力矩——即纯重力补偿力矩。

```python
gc = NeroGravityCompensator()  # 自动加载内置 URDF
tau_gravity = gc.compute(q)     # 输入 7 维关节角度，返回 7 维重力矩
```

**URDF 说明：**

使用 `robot_models/nero/nero_description.urdf`。加载时代码会用正则自动剥离所有 `<visual>` 和 `<collision>` 标签，得到纯动力学模型，无需解析 STL mesh 路径。

**安全阈值：**

`max_torque_sanity=20.0`：如果计算出的任何关节重力矩绝对值超过 20 N·m，抛出 `RuntimeError`。这是一个配置错误保护——正常 Nero 臂（总重约 4 kg，最长力臂约 0.58 m）的最大重力矩约 `4 × 9.81 × 0.58 ≈ 22.8 N·m`，但只在完全水平伸展时才接近该值。如果末端挂载较重工具，可能需要调高此值。

---

### 4.3 mit_loop.py — 实时控制循环

**文件**: `force_ctrl/mit_loop.py`
**类**: `NeroMITLoop`

#### 控制循环单次迭代流程

```
loop_start = perf_counter()
│
├─ 1. q = robot.get_joint_angles()          # 从 parser 缓存读取（无阻塞）
│     如果 None → miss_count++，超限则急停
│
├─ 2. 位置跳变检测
│     |q - q_prev| > 0.5 rad → 警告日志
│
├─ 3. 关节限位检查
│     q 超出 limits ± 0.1 rad buffer → 急停
│
├─ 4. grav_torque = mujoco.mj_inverse(q) × gravity_factor
│
├─ 5. cmds = deepcopy(user_commands)        # 加锁读取
│
├─ 6. total_torque = cmds.torques + grav_torque
│
├─ 7. total_torque = clip(total_torque, ±torque_limit)
│
├─ 8. cmd_pos = clip(cmds.pos, joint_limits)
│
├─ 9. move_mit_batch(pos, vel, kp, kd, total_torque)  # 发 7 帧 CAN
│
├─ 10. 更新 joint_state（位置 + 速度 + 力矩）
│
└─ 11. sleep_until(loop_start + period)     # 混合等待: sleep + busy-wait
```

#### 安全机制

| 检查项 | 触发条件 | 响应 |
|--------|---------|------|
| 通信心跳 | 连续 20 个周期（100ms）无关节反馈 | 急停 + 停循环 |
| 关节限位 | 实际角度超出 limits ± 0.1 rad | 急停 + 停循环 |
| 位置跳变 | 相邻帧角度差 > 0.5 rad | 警告日志（不停止） |
| 力矩 clamp | 输出力矩超出 ±torque_limit | 截断到限制值 |
| 重力矩异常 | mj_inverse 输出 > 20 N·m | RuntimeError → 急停 |
| 位置 clamp | 位置命令超出关节限位 | 截断到限位值 |

#### 限速机制

```python
def _sleep_until(self, target_time):
    remaining = target_time - perf_counter()
    if remaining > 0.002:
        time.sleep(remaining - 0.001)   # 粗等待
    while perf_counter() < target_time:  # 最后 ~1ms busy-wait 保精度
        pass
```

每 30 秒打印一次循环频率统计日志，报告实际频率和超时比例。

---

### 4.4 controller.py — 高层接口

**文件**: `force_ctrl/controller.py`
**类**: `NeroForceController`

#### 构造参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `channel` | `"can0"` | CAN 通道名 |
| `urdf_path` | `""` (自动加载内置) | Nero URDF 路径 |
| `zero_gravity` | `True` | 启动后初始模式：True=零重力，False=位置保持 |
| `gravity_factor` | `1.3` | 重力补偿系数（>1 补偿摩擦） |
| `control_freq` | `200` | 控制循环频率 (Hz) |
| `torque_limit` | `8.0` | 单关节最大力矩 (N·m) |
| `interface` | `"socketcan"` | CAN 接口类型 |

#### 生命周期

```python
# 方式一: 手动管理
ctrl = NeroForceController(channel="can0")
ctrl.start()    # connect → enable → 等待首帧反馈 → 启动控制循环
# ... 使用 ...
ctrl.stop()     # 停循环 → 急停 → disable

# 方式二: context manager
with NeroForceController(channel="can0") as ctrl:
    # ... 使用 ...
    pass  # 自动 stop
```

`start()` 内部执行的完整序列：
1. `robot.connect()` — 初始化 CAN 通信，启动读线程
2. `robot.set_normal_mode()` — 设置单臂控制模式
3. `robot.enable()` — 循环使能所有关节（超时 5s）
4. 等待首帧关节反馈（超时 3s）
5. 设置初始 JointCommands（零重力 or 位置保持）
6. `NeroMITLoop.start()` — 启动 200Hz 后台控制线程

#### 控制模式 API

**零重力拖动 `zero_gravity_mode()`**

```
MIT 命令:  pos=当前位置, vel=0, kp=0, kd=ZERO_GRAV_KD, t_ff=仅重力补偿
效果:      机械臂漂浮，可自由拖动；小阻尼防止震荡
```

`ZERO_GRAV_KD = [1.0, 1.0, 1.0, 0.5, 0.3, 0.2, 0.2]`

**位置 PD 控制 `command_joint_pos(pos, kp, kd)`**

```
MIT 命令:  pos=目标位置, vel=0, kp=DEFAULT_KP, kd=DEFAULT_KD, t_ff=重力补偿
效果:      关节保持目标位置，施加外力时柔顺回弹
```

`DEFAULT_KP = [30, 30, 30, 15, 10, 5, 5]`
`DEFAULT_KD = [2.0, 2.0, 2.0, 1.0, 0.5, 0.3, 0.3]`

**全参数 MIT 控制 `command_joint_state(pos, vel, kp, kd, torques)`**

```
MIT 命令:  pos, vel, kp, kd 由用户指定; t_ff = user_torques + 重力补偿
效果:      完全自定义阻抗行为
```

**零力矩模式 `zero_torque_mode()`**

```
MIT 命令:  pos=0, vel=0, kp=0, kd=0, t_ff=0（无重力补偿!）
效果:      机械臂完全被动，会自由落下。仅用于调试。
```

#### MIT 控制律（电机固件执行）

每个关节的电机固件内部执行：

```
τ_output = kp × (p_des − p_actual) + kd × (v_des − v_actual) + t_ff
```

其中 `t_ff` 已经包含了重力补偿（由本模块的控制循环叠加）。

---

### 4.5 driver.py 修改 — move_mit / move_mit_batch

**文件**: `nero/default/driver.py`

#### `set_motion_mode()` 扩展

类型标注从 `Literal['p', 'j']` 扩展为 `Literal['p', 'j', 'mit']`。
运行时校验逻辑不变（本来就接受 `'mit'`）。

#### `move_mit(joint_index, p_des, v_des, kp, kd, t_ff)`

控制单个关节。流程：
1. 校验 joint_index (1~7)
2. 校验并 clamp 五个参数到各自范围
3. `FloatToUint` 量化为定点整数
4. `_make_joint_mit_ctrl_msg` 构建 CAN 消息
5. `set_motion_mode('mit')` + `_send_msg()`

#### `move_mit_batch(commands)`

批量发送多关节 MIT 命令。流程：
1. 仅调用一次 `set_motion_mode('mit')`（避免每个关节都切模式）
2. 遍历 commands 列表，逐关节校验、量化、构建、发送

参数量化范围和精度：

| 参数 | 浮点范围 | 量化位数 | 精度 |
|------|---------|---------|------|
| `p_des` | [-12.5, 12.5] rad | 16 bit | 3.8e-4 rad |
| `v_des` | [-45.0, 45.0] rad/s | 12 bit | 2.2e-2 rad/s |
| `kp` | [0.0, 500.0] | 12 bit | 1.2e-1 |
| `kd` | [-5.0, 5.0] | 12 bit | 2.4e-3 |
| `t_ff` | [-8.0, 8.0] N·m | 8 bit | 6.3e-2 N·m |

**注意**: `t_ff` 仅有 8 bit 精度（256 级），约 0.063 N·m 步进。对于精细力控场景，这可能是精度瓶颈。

#### CAN 帧格式（每关节 8 字节）

```
Byte 0:    p_des[15:8]
Byte 1:    p_des[7:0]
Byte 2:    v_des[11:4]
Byte 3:    v_des[3:0] | kp[11:8]
Byte 4:    kp[7:0]
Byte 5:    kd[11:4]
Byte 6:    kd[3:0] | t_ff[7:4]
Byte 7:    t_ff[3:0] | crc[3:0]      (4-bit XOR CRC)
```

CAN ID 映射：

| 关节 | CAN ID |
|------|--------|
| 1 | 0x15A |
| 2 | 0x15B |
| 3 | 0x15C |
| 4 | 0x15D |
| 5 | 0x15E |
| 6 | 0x15F |
| 7 | 0x160 |

---

## 5. 关节限位参考

| 关节 | 最小 (rad) | 最大 (rad) | 最小 (deg) | 最大 (deg) |
|------|-----------|-----------|-----------|-----------|
| joint1 | -2.705 | 2.705 | -155.0 | 155.0 |
| joint2 | -1.745 | 1.745 | -100.0 | 100.0 |
| joint3 | -2.758 | 2.758 | -158.0 | 158.0 |
| joint4 | -1.012 | 2.147 | -58.0 | 123.0 |
| joint5 | -2.758 | 2.758 | -158.0 | 158.0 |
| joint6 | -0.733 | 0.960 | -42.0 | 55.0 |
| joint7 | -1.571 | 1.571 | -90.0 | 90.0 |

---

## 6. 依赖

```
python-can>=3.3.4       # 已有（pyAgxArm 依赖）
typing-extensions>=3.7  # 已有（pyAgxArm 依赖）
numpy                   # 已有
mujoco>=3.0             # 新增（pip install mujoco）
```

---

## 7. 使用示例

### 7.1 零重力拖动

```python
from pyAgxArm.protocols.can_protocol.drivers.nero.force_ctrl import (
    NeroForceController,
)

with NeroForceController(channel="can0", zero_gravity=True) as ctrl:
    input("零重力模式，可以拖动机械臂。按 Enter 退出...")
```

### 7.2 位置保持

```python
import numpy as np
from pyAgxArm.protocols.can_protocol.drivers.nero.force_ctrl import (
    NeroForceController,
)

ctrl = NeroForceController(channel="can0", zero_gravity=False)
ctrl.start()

# 移动到指定关节角度
target = np.array([0.0, 0.3, -0.3, 0.5, 0.0, 0.0, 0.0])
ctrl.command_joint_pos(target)

input("位置保持中，按 Enter 退出...")
ctrl.stop()
```

### 7.3 柔顺控制（低刚度）

```python
q = ctrl.get_joint_pos()
ctrl.command_joint_state(
    pos=q,
    vel=np.zeros(7),
    kp=np.array([5, 5, 5, 3, 2, 1, 1]),       # 低刚度
    kd=np.array([2, 2, 2, 1, 0.5, 0.3, 0.3]),
    torques=np.zeros(7),                         # 重力补偿自动叠加
)
```

### 7.4 纯力矩控制

```python
# 只施加重力补偿 + 自定义前馈力矩，无位置/速度反馈
q = ctrl.get_joint_pos()
ctrl.command_joint_state(
    pos=q,
    vel=np.zeros(7),
    kp=np.zeros(7),      # 不用位置环
    kd=np.zeros(7),      # 不用速度环
    torques=my_torques,   # 自定义力矩（重力补偿自动叠加）
)
```

### 7.5 底层单关节 MIT（不经过控制循环）

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="nero", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
robot.connect()
robot.set_normal_mode()
while not robot.enable():
    pass

# 单关节 MIT 阻抗控制
robot.move_mit(joint_index=1, p_des=0.5, kp=10.0, kd=0.8, t_ff=0.0)

# 批量 7 关节 MIT
robot.move_mit_batch([
    {"joint_index": i, "p_des": 0.0, "kp": 10.0, "kd": 0.8}
    for i in range(1, 8)
])
```

### 7.6 Demo 脚本

```bash
# 零重力模式
python -m pyAgxArm.demos.nero.force_ctrl_demo --channel can0 --mode zero_gravity --kd_scale 0.0001

# 位置保持模式
python -m pyAgxArm.demos.nero.force_ctrl_demo --channel can0 --mode position

# 柔顺模式
python -m pyAgxArm.demos.nero.force_ctrl_demo --channel can0 --mode compliant

# 自定义参数
python -m pyAgxArm.demos.nero.force_ctrl_demo \
    --channel can0 \
    --mode zero_gravity \
    --gravity_factor 1.5 \
    --freq 100
```

---

## 8. 实机调试指南

### 8.1 首次上电检查清单

1. **CAN 总线**: `sudo ip link set can0 up type can bitrate 1000000`
2. **机械臂上电**: 确认电源指示灯正常
3. **通信验证**: `candump can0` 应能看到反馈帧（0x2A5 等）
4. **安装 mujoco**: `pip install mujoco`

### 8.2 gravity_factor 调参

`gravity_factor` 是重力补偿力矩的乘数。默认 1.3，意味着输出 130% 的理论重力矩。

| 现象 | 原因 | 调整 |
|------|------|------|
| 零重力模式下机械臂缓慢下坠 | 补偿不足 | 增大 gravity_factor（试 1.4~1.6） |
| 零重力模式下机械臂自动向上抬 | 补偿过度 | 减小 gravity_factor（试 1.1~1.2） |
| 某些姿态补偿好、某些姿态偏 | URDF 惯性参数不准 | 校准 URDF 的质量/质心 |

**调试步骤：**
1. 启动零重力模式
2. 把机械臂摆到水平伸展姿态（重力矩最大）
3. 松手观察：应静止不动
4. 如果下坠，增大 factor；如果上抬，减小 factor
5. 多个姿态都确认后锁定值

### 8.3 kp / kd 调参

默认值 `DEFAULT_KP = [30, 30, 30, 15, 10, 5, 5]` 是保守起步值。

**通用原则：**
- 大关节（1-3）需要更大 kp（承载重力矩大）
- 腕部关节（4-7）kp 可以较小（惯量小）
- kd 用于阻尼，太小会震荡，太大会迟钝
- 柔顺任务用低 kp（1~10），刚性定位用高 kp（50~100）

**调试步骤：**
1. 从低 kp 开始（如 [5,5,5,3,2,1,1]），确认无震荡
2. 逐步增大 kp 直到刚度满足需求
3. 如果出现震荡，增大 kd 或减小 kp
4. kd 的范围是 [-5.0, 5.0]，一般正值在 [0.1, 3.0] 之间

### 8.4 torque_limit 设置

默认 8.0 N·m。这是 MIT CAN 帧 `t_ff` 字段的硬件范围上限（[-8.0, 8.0]）。

如果挂载重工具导致重力补偿力矩超过 8 N·m，需要确认 Nero 主控固件是否支持更大的力矩范围。如果支持，也需要同步修改 `_validate_and_quantize_mit_params` 中的 clamp 范围。

### 8.5 控制频率

默认 200 Hz（5ms 周期），每个周期发 7 帧 MIT + 偶尔 1 帧模式控制。

CAN 总线带宽估算：
- MIT 帧: 7 × 200 = 1400 帧/s
- 标准 CAN 帧（11-bit ID, 8 byte data）约 130 bit/帧
- 带宽: 1400 × 130 ≈ 182 kbps（1 Mbps 的 18%）

**读帧也占带宽**（pyAgxArm 读线程）：
- 反馈帧: 约 21 帧 × ~200 Hz = ~4200 帧/s
- 额外带宽: ~546 kbps

总计约 **730 kbps / 1 Mbps**，占比 73%。如果出现丢帧，可降低 control_freq 到 100~150 Hz。

### 8.6 日志

本模块使用 Python 标准 `logging`。开启方法：

```python
import logging
logging.basicConfig(level=logging.INFO)
# 或者只看力控模块:
logging.getLogger("pyAgxArm.protocols.can_protocol.drivers.nero.force_ctrl").setLevel(logging.DEBUG)
```

关键日志输出：
- `INFO`: 启停状态、控制频率统计（每 30s）
- `WARNING`: 位置跳变、急停
- `ERROR`: 限位违规、通信丢失、力矩异常

### 8.7 常见问题排查

| 问题 | 可能原因 | 排查方法 |
|------|---------|---------|
| `start()` 超时: "Failed to enable" | CAN 未连接 / 臂未上电 / 急停未解除 | `candump can0` 看有无反馈帧 |
| 零重力模式下剧烈抖动 | kd 太小 / gravity_factor 不对 | 增大 ZERO_GRAV_KD 或调 factor |
| 位置保持模式有静差 | kp 不够大 / 重力补偿不准 | 增大 kp 或调 gravity_factor |
| 控制频率统计远低于 200 Hz | Python GIL 竞争 / CPU 负载高 | 降低 freq，或用 `htop` 查 CPU |
| "Joint limit exceeded, emergency stop" | 机械臂被外力推到极限位 | 检查限位 buffer 是否够大 |
| "Gravity compensation torque too large" | URDF 参数与实际不符 | 对比 URDF 质量和实际称重 |

---

## 9. 后续扩展方向

1. **笛卡尔空间力控**: 在 `NeroForceController` 上层增加 task-space 阻抗控制器（已有 `nero_urdf_admittance_v2` 中的 `NeroElbowAdmittanceController` 可参考）
2. **力/力矩传感器集成**: 读取末端 F/T 传感器数据，接入控制循环实现闭环力控
3. **碰撞检测**: 利用电机电流反馈做无传感器碰撞检测
4. **可变阻抗**: 根据任务阶段动态调整 kp/kd（如接近时低阻抗、接触后高阻抗）
5. **轨迹跟踪**: 在控制循环内加入在线轨迹生成（如 Ruckig jerk-limited）
