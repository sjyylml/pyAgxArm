# revo2 API使用文档

## 创建实例

### 创建机械臂Driver实例 AgxArmFactory.create_arm()

- 使用示例:

robot 形参范围：["nero", "piper", "piper_h", "piper_l", "piper_x"]

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="nero", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
```

### 创建末端执行器Driver实例 robot.init_effector()

- 功能说明:

根据传入的末端执行器种类，创建对应的执行器实例

- 定义:

```python
init_effector(self, effector: str) -> EffectorDriver
```

- 参数说明:

|名称|类型|说明|
|---|---|---|
|effector|str|初始化指定的执行器|

可以通过机械臂Driver内部的`EFFECTOR`类中的预定义变量来作为输入的实参，定义如下：

```python
class EFFECTOR:
    """
    End-effector kind constants.

    Use:
        robot.init_effector(robot.OPTIONS.EFFECTOR.AGX_GRIPPER)
    """

    AGX_GRIPPER: Final[Literal["agx_gripper"]] = "agx_gripper"
    REVO2: Final[Literal["revo2"]] = "revo2"
```

- 返回值:

类型:EffectorDriver

不同的输入参数effector，返回对应的Driver

- 使用示例:

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="nero", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
end_effector = robot.init_effector(robot.OPTIONS.EFFECTOR.REVO2)
```

注意：

1. init_effector函数只能执行一次，无法初始化两个执行器
2. 最好在连接前进行执行器的创建

### 创建连接 connect()

- 功能说明:

该指令用来创建连接，启动读取线程

- 定义:

```python
connect(self, start_read_thread: bool = True) -> None
```

- 参数说明:

|名称|类型|说明|
|---|---|---|
|start_read_thread|bool|是否打开读取数据线程，默认为打开|

- 使用示例:

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="nero", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
end_effector = robot.init_effector(robot.OPTIONS.EFFECTOR.REVO2)
robot.connect()
```

## 通用状态相关

### 通信是否正常 is_ok()

- 功能说明:

用于判断末端执行器数据接收是否正常。该值由 SDK 内部的数据监控逻辑根据“最近一段时间是否持续收不到数据”计算得出。

- 定义:

```python
is_ok(self) -> bool
```

- 返回值:

类型：`bool`

- 使用示例:

```python
import time
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="nero", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
end_effector = robot.init_effector(robot.OPTIONS.EFFECTOR.REVO2)
robot.connect()

time.sleep(0.5)
print("effector is_ok =", end_effector.is_ok())
```

### 获取执行器数据接收频率 get_fps()

- 功能说明:

获取末端执行器数据监控的接收频率（Hz）。该频率是 SDK 对执行器解析器收到数据的统计值。

- 定义:

```python
get_fps(self) -> float
```

- 返回值:

类型：`float`（单位：Hz）

- 使用示例:

```python
import time
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="nero", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
end_effector = robot.init_effector(robot.OPTIONS.EFFECTOR.REVO2)
robot.connect()

time.sleep(0.5)
print("effector fps =", end_effector.get_fps(), "Hz")
```

## 执行器读取数据相关

### MessageAbstract 返回值通用说明（重要）

本 SDK 多数读取接口返回 `MessageAbstract[T] | None`。其中：

|字段|类型|说明|
|---|---|---|
|`ret.msg`|`T`|消息数据本体（例如灵巧手状态/各指位置/速度等结构体）|
|`ret.hz`|`float`|该消息类型的接收频率（由 SDK 统计），单位：Hz|
|`ret.timestamp`|`float`|消息时间戳（由 SDK 记录），单位：s|

### 读取末端执行器状态 get_hand_status()

- 功能说明:

读取灵巧手各指运行状态反馈（左右手标志、各指电机状态：空闲/运行/堵转）。

- 定义:

```python
get_hand_status(self) -> Optional[MessageAbstract[FeedbackHandStatus]]
```

- 返回值:

类型：`MessageAbstract[FeedbackHandStatus] | None`

- 消息字段说明（`MessageAbstract[FeedbackHandStatus].msg`）:

|字段|类型|说明|
|---|---|---|
|left_or_right|int|左右手标志：01 左手；02 右手|
|thumb_tip|int|拇指尖电机状态：0 空闲；1 运行；2 堵转|
|thumb_base|int|拇指根电机状态：0 空闲；1 运行；2 堵转|
|index_finger|int|食指电机状态：0 空闲；1 运行；2 堵转|
|middle_finger|int|中指电机状态：0 空闲；1 运行；2 堵转|
|ring_finger|int|无名指电机状态：0 空闲；1 运行；2 堵转|
|pinky_finger|int|小指电机状态：0 空闲；1 运行；2 堵转|

- 使用示例:

```python
import time
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="nero", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
end_effector = robot.init_effector(robot.OPTIONS.EFFECTOR.REVO2)
robot.connect()

time.sleep(0.2)
hs = end_effector.get_hand_status()
if hs is not None:
    print(hs.msg)
    print(hs.hz, hs.timestamp)
```

### 读取各指位置 get_finger_pos()

- 功能说明:

读取灵巧手各指位置反馈（0~100）。

- 定义:

```python
get_finger_pos(self) -> Optional[MessageAbstract[FeedbackFingerPos]]
```

- 返回值:

类型：`MessageAbstract[FeedbackFingerPos] | None`

- 消息字段说明（`MessageAbstract[FeedbackFingerPos].msg`）:

|字段|类型|说明|
|---|---|---|
|thumb_tip|int|拇指尖位置，范围：[0, 100]|
|thumb_base|int|拇指根位置，范围：[0, 100]|
|index_finger|int|食指位置，范围：[0, 100]|
|middle_finger|int|中指位置，范围：[0, 100]|
|ring_finger|int|无名指位置，范围：[0, 100]|
|pinky_finger|int|小指位置，范围：[0, 100]|

- 使用示例:

```python
import time
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="nero", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
end_effector = robot.init_effector(robot.OPTIONS.EFFECTOR.REVO2)
robot.connect()

time.sleep(0.2)
fp = end_effector.get_finger_pos()
if fp is not None:
    print(fp.msg)
    print(fp.hz, fp.timestamp)
```

### 读取各指速度 get_finger_spd()

- 功能说明:

读取灵巧手各指速度反馈。

- 定义:

```python
get_finger_spd(self) -> Optional[MessageAbstract[FeedbackFingerSpd]]
```

- 返回值:

类型：`MessageAbstract[FeedbackFingerSpd] | None`

- 消息字段说明（`MessageAbstract[FeedbackFingerSpd].msg`）:

|字段|类型|说明|
|---|---|---|
|thumb_tip|int|拇指尖速度，范围：[-100, 100]|
|thumb_base|int|拇指根速度，范围：[-100, 100]|
|index_finger|int|食指速度，范围：[-100, 100]|
|middle_finger|int|中指速度，范围：[-100, 100]|
|ring_finger|int|无名指速度，范围：[-100, 100]|
|pinky_finger|int|小指速度，范围：[-100, 100]|

- 使用示例:

```python
import time
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="nero", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
end_effector = robot.init_effector(robot.OPTIONS.EFFECTOR.REVO2)
robot.connect()

time.sleep(0.2)
fs = end_effector.get_finger_spd()
if fs is not None:
    print(fs.msg)
    print(fs.hz, fs.timestamp)
```

### 读取各指电流 get_finger_current()

- 功能说明:

读取灵巧手各指电流反馈。

- 定义:

```python
get_finger_current(self) -> Optional[MessageAbstract[FeedbackFingerCurrent]]
```

- 返回值:

类型：`MessageAbstract[FeedbackFingerCurrent] | None`

- 消息字段说明（`MessageAbstract[FeedbackFingerCurrent].msg`）:

|字段|类型|说明|
|---|---|---|
|thumb_tip|int|拇指尖电流，范围：[-100, 100]|
|thumb_base|int|拇指根电流，范围：[-100, 100]|
|index_finger|int|食指电流，范围：[-100, 100]|
|middle_finger|int|中指电流，范围：[-100, 100]|
|ring_finger|int|无名指电流，范围：[-100, 100]|
|pinky_finger|int|小指电流，范围：[-100, 100]|

- 使用示例:

```python
import time
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="nero", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
end_effector = robot.init_effector(robot.OPTIONS.EFFECTOR.REVO2)
robot.connect()

time.sleep(0.2)
fc = end_effector.get_finger_current()
if fc is not None:
    print(fc.msg)
    print(fc.hz, fc.timestamp)
```

## 执行器控制相关

### 控制执行器 position_ctrl()

- 功能说明:

按“位置模式”控制各手指目标位置。

- 定义:

```python
position_ctrl(
    self,
    thumb_tip: int = 0,
    thumb_base: int = 0,
    index_finger: int = 0,
    middle_finger: int = 0,
    ring_finger: int = 0,
    pinky_finger: int = 0,
) -> None
```

- 参数说明:

各参数均为目标位置，范围：[0, 100]：

|名称|类型|说明|
|---|---|---|
|thumb_tip|int|拇指尖位置|
|thumb_base|int|拇指根位置|
|index_finger|int|食指位置|
|middle_finger|int|中指位置|
|ring_finger|int|无名指位置|
|pinky_finger|int|小指位置|

- 使用示例:

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="nero", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
end_effector = robot.init_effector(robot.OPTIONS.EFFECTOR.REVO2)
robot.connect()

# 全部张开/归零
end_effector.position_ctrl()

# 拇指尖到 100，其余不变（示例）
end_effector.position_ctrl(thumb_tip=100)
```

### 控制执行器 speed_ctrl()

- 功能说明:

按“速度模式”控制各手指目标速度。

- 定义:

```python
speed_ctrl(
    self,
    thumb_tip: int = 0,
    thumb_base: int = 0,
    index_finger: int = 0,
    middle_finger: int = 0,
    ring_finger: int = 0,
    pinky_finger: int = 0,
) -> None
```

- 参数说明:

各参数均为目标速度，范围：[-100, 100]。

- 使用示例:

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="nero", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
end_effector = robot.init_effector(robot.OPTIONS.EFFECTOR.REVO2)
robot.connect()

end_effector.speed_ctrl(thumb_tip=50)
```

### 控制执行器 current_ctrl()

- 功能说明:

按“电流模式”控制各手指目标电流。

- 定义:

```python
current_ctrl(
    self,
    thumb_tip: int = 0,
    thumb_base: int = 0,
    index_finger: int = 0,
    middle_finger: int = 0,
    ring_finger: int = 0,
    pinky_finger: int = 0,
) -> None
```

- 参数说明:

各参数均为目标电流，范围：[-100, 100]。

- 使用示例:

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="nero", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
end_effector = robot.init_effector(robot.OPTIONS.EFFECTOR.REVO2)
robot.connect()

end_effector.current_ctrl(thumb_tip=10)
```

### 控制执行器 position_time_ctrl()

- 功能说明:

位置/时间混合控制：先用 `mode="pos"` 下发目标位置，再用 `mode="time"` 下发到位时间（单位 10ms）。注意两条消息的间隔不应超过 50ms。

- 定义:

```python
position_time_ctrl(
    self,
    mode: Literal['pos', 'time'] = 'pos',
    thumb_tip: int = 0,
    thumb_base: int = 0,
    index_finger: int = 0,
    middle_finger: int = 0,
    ring_finger: int = 0,
    pinky_finger: int = 0,
) -> None
```

- 参数说明:

|名称|类型|说明|
|---|---|---|
|mode|str|控制模式：`pos` 位置（0 ~ 100）；`time` 时间（单位 10ms，范围 0~255）|
|thumb_tip~pinky_finger|int|在 `pos` 模式下为目标位置；在 `time` 模式下为目标时间|

- 使用示例:

```python
import time
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="nero", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
end_effector = robot.init_effector(robot.OPTIONS.EFFECTOR.REVO2)
robot.connect()

# 拇指尖移动到位置 100，然后设置 2 秒到位（200 * 10ms）
end_effector.position_time_ctrl(mode="pos", thumb_tip=100)
end_effector.position_time_ctrl(mode="time", thumb_tip=200)
```
