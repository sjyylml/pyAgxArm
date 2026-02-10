# AgxGripper API使用文档

## 创建实例

### 创建机械臂Driver实例 AgxArmFactory.create_arm()

- 使用示例:

`robot` 形参范围（robotic arm 系列）：["nero", "piper", "piper_h", "piper_l", "piper_x"]

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
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

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
end_effector = robot.init_effector(robot.OPTIONS.EFFECTOR.AGX_GRIPPER)
```

注意：

1. init_effector函数只能执行一次，无法初始化两个执行器
2. 最好在连接前进行执行器的创建
3. init_effector函数执行后会返回Driver实例，外部使用时需要创建一个外部的新变量来获取，然后可以用新创建的变量来调用执行器Driver内部的方法

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

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
end_effector = robot.init_effector(robot.OPTIONS.EFFECTOR.AGX_GRIPPER)
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

- 说明:

1. 一般在 `robot.connect()` 并启动读线程后才有意义
2. 若最近多个监控周期内接收频率持续为 0，`is_ok()` 可能返回 `False`

- 使用示例:

```python
import time
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
end_effector = robot.init_effector(robot.OPTIONS.EFFECTOR.AGX_GRIPPER)
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

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
end_effector = robot.init_effector(robot.OPTIONS.EFFECTOR.AGX_GRIPPER)
robot.connect()

time.sleep(0.5)
print("effector fps =", end_effector.get_fps(), "Hz")
```

## 执行器读取数据相关

### MessageAbstract 返回值通用说明（重要）

本 SDK 多数读取接口返回 `MessageAbstract[T] | None`。其中：

|字段|类型|说明|
|---|---|---|
|`ret.msg`|`T`|消息数据本体（例如夹爪状态结构体/参数结构体等）|
|`ret.hz`|`float`|该消息类型的接收频率（由 SDK 统计），单位：Hz|
|`ret.timestamp`|`float`|消息时间戳（由 SDK 记录），单位：s|

### 读取末端执行器状态 get_gripper_status()

- 功能说明:

读取 AgxGripper 的实时状态反馈（当前夹爪开口宽度、夹持力、驱动状态等）。

- 定义:

```python
get_gripper_status(self) -> Optional[MessageAbstract[ArmMsgFeedbackGripper]]
```

- 返回值:

类型：`MessageAbstract[ArmMsgFeedbackGripper] | None`

- 返回 `None` 的常见原因：

1. 尚未 `robot.connect()` 启动读线程，或 `connect(start_read_thread=False)` 未启动读取线程
2. 刚连接还没收到夹爪反馈帧（建议循环等待一小段时间）
3. 当前 CAN 总线上没有该夹爪的反馈数据

- 消息字段说明（`MessageAbstract[ArmMsgFeedbackGripper].msg`）:

|字段|类型|说明|
|---|---|---|
|width|float|当前夹爪开口宽度，单位：m|
|force|float|当前夹持力，单位：N|
|foc_status|object|驱动状态集合（见下表）|

`foc_status` 子字段（`gs.msg.foc_status.xxx`）：

|字段|类型|说明|
|---|---|---|
|voltage_too_low|bool|电压过低|
|motor_overheating|bool|电机过温|
|driver_overcurrent|bool|驱动过流|
|driver_overheating|bool|驱动过温|
|sensor_status|bool|传感器状态|
|driver_error_status|bool|驱动错误状态|
|driver_enable_status|bool|驱动使能状态|
|homing_status|bool|回零/零位状态|

- 使用示例:

```python
import time
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
end_effector = robot.init_effector(robot.OPTIONS.EFFECTOR.AGX_GRIPPER)
robot.connect()

while True:
    gs = end_effector.get_gripper_status()
    if gs is not None:
        print("width(m)=", gs.msg.width, "force(N)=", gs.msg.force)
        print("hz=", gs.hz, "timestamp=", gs.timestamp)
        break
    time.sleep(0.05)
```

### 读取末端执行器控制消息 get_gripper_ctrl_states()

- 功能说明:

读取夹爪控制指令的反馈/回显状态（当前宽度、力、状态码、回零指令回显等）。

- 定义:

```python
get_gripper_ctrl_states(self) -> Optional[MessageAbstract[ArmMsgGripperCtrl]]
```

- 返回值:

类型：`MessageAbstract[ArmMsgGripperCtrl] | None`

- 返回 `None` 的常见原因：

1. 未连接或未启动读线程
2. 尚未收到夹爪控制反馈帧（建议循环等待）

- 消息字段说明（`MessageAbstract[ArmMsgGripperCtrl].msg`）:

|字段|类型|说明|
|---|---|---|
|width|float|当前夹爪开口宽度，单位：m|
|force|float|当前夹持力，单位：N|
|status_code|int|状态码（与控制指令相关）|
|set_zero|int|回零/置零相关字段（与控制指令相关）|

- 使用示例:

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
end_effector = robot.init_effector(robot.OPTIONS.EFFECTOR.AGX_GRIPPER)
robot.connect()

gcs = end_effector.get_gripper_ctrl_states()
if gcs is not None:
    print("width(m)=", gcs.msg.width, "force(N)=", gcs.msg.force)
    print("status_code=", gcs.msg.status_code, "set_zero=", gcs.msg.set_zero)
    print("hz=", gcs.hz, "timestamp=", gcs.timestamp)
```

### 读取夹爪/示教器参数 get_gripper_teaching_pendant_param()

- 功能说明:

查询夹爪示教器相关参数（如示教范围、最大行程配置、示教摩擦等）。该接口会下发查询帧并等待对应反馈。

- 定义:

```python
get_gripper_teaching_pendant_param(
    self, timeout: float = 1.0, min_interval: float = 1.0
) -> Optional[MessageAbstract[ArmMsgFeedbackGripperTeachingPendantParam]]
```

- 参数说明:

|名称|类型|说明|
|---|---|---|
|timeout|float|等待反馈超时时间（秒），默认 1.0|
|min_interval|float|最小请求间隔（秒），默认 1.0|

- 返回值:

类型：`MessageAbstract[ArmMsgFeedbackGripperTeachingPendantParam] | None`

- 消息字段说明（`MessageAbstract[ArmMsgFeedbackGripperTeachingPendantParam].msg`）:

|字段|类型|说明|
|---|---|---|
|teaching_range_per|int|示教范围（百分比式值），范围：[100, 200]，仅适用于主从臂的**主臂**：用于放大主臂的控制行程并映射给从臂|
|max_range_config|float|最大行程配置，单位：m，常见值：0 / 0.07 / 0.1|
|teaching_friction|int|示教摩擦，范围：1..10，**值越大越灵敏**：更容易张合夹爪，所需用力更小|

- 使用示例:

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
end_effector = robot.init_effector(robot.OPTIONS.EFFECTOR.AGX_GRIPPER)
robot.connect()

param = end_effector.get_gripper_teaching_pendant_param()
if param is not None:
    print(
        param.msg.teaching_range_per,
        param.msg.max_range_config,
        param.msg.teaching_friction,
    )
    print(param.hz, param.timestamp)
```

## 执行器控制相关

### 控制执行器 move_gripper()

- 功能说明:

控制夹爪运动到目标开口宽度，并设置目标夹持力。

- 定义:

```python
move_gripper(self, width: float = 0.0, force: float = 1.0) -> None
```

- 参数说明:

|名称|类型|说明|
|---|---|---|
|width|float|目标夹爪开口宽度，单位：m，范围：[0.0, 0.1]，默认：0.0（数值精度约 \(1e^{-6}\) m）|
|force|float|目标夹持力，单位：N，范围：[0.0, 3.0]，默认：1.0（数值精度约 \(1e^{-3}\) N）|

- 异常:

|异常|触发条件|
|---|---|
|ValueError|`width` 不在 [0.0, 0.1] m 或 `force` 不在 [0.0, 3.0] N|

- 使用示例:

```python
import time
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
end_effector = robot.init_effector(robot.OPTIONS.EFFECTOR.AGX_GRIPPER)
robot.connect()

# 张开到 5cm，力 1N
end_effector.move_gripper(width=0.05, force=1.0)
time.sleep(1.0)

# 闭合（宽度 0）
end_effector.move_gripper(width=0.0, force=1.0)
```

注意：

1. 建议在 `robot.connect()` 后再下发控制指令，确保通信与读线程正常工作
2. `width/force` 单位分别是 **米(m)** / **牛(N)**，不要直接输入毫米或克力等单位

### 禁用夹爪 disable_gripper()

- 功能说明:

禁用夹爪驱动（相当于下发“驱动失能”指令）。该接口会通过反馈推断是否禁用成功。

- 定义:

```python
disable_gripper(self) -> bool
```

- 返回值:

类型：`bool`（`True` 表示判断为禁用成功，`False` 表示未成功或无法判断）

- 注意:

该接口会在下发指令后读取一次 `get_gripper_status()` 来判断驱动使能位。若当前没有状态反馈（返回 `None`），则会返回 `False`；必要时可稍作等待后再调用一次。

- 使用示例:

```python
import time
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
end_effector = robot.init_effector(robot.OPTIONS.EFFECTOR.AGX_GRIPPER)
robot.connect()

time.sleep(0.2)
if end_effector.disable_gripper():
    print("Gripper disabled")
```

### 夹爪置零/标定 calibrate_gripper()

- 功能说明:

将当前位置设置为夹爪零点（置零/标定）。该接口会等待控制器返回 ACK/响应并判定是否成功。

- 定义:

```python
calibrate_gripper(self, timeout: float = 1.0) -> bool
```

- 参数说明:

|名称|类型|说明|
|---|---|---|
|timeout|float|等待响应超时时间（秒），默认 1.0|

- 返回值:

类型：`bool`（在 `timeout` 内收到成功响应则为 `True`，否则 `False`）

- 使用示例:

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
end_effector = robot.init_effector(robot.OPTIONS.EFFECTOR.AGX_GRIPPER)
robot.connect()

end_effector.disable_gripper()
input("Please move the gripper to the zero position...")
if end_effector.calibrate_gripper():
    end_effector.move_gripper(width=0.0)
```

### 配置夹爪/示教器参数 set_gripper_teaching_pendant_param()

- 功能说明:

配置夹爪示教器相关参数，并在必要时通过再次查询来校验配置是否生效。

- 定义:

```python
set_gripper_teaching_pendant_param(
    self,
    teaching_range_per: int = 100,
    max_range_config: float = 0.0,
    teaching_friction: int = 1,
    timeout: float = 1.0,
) -> bool
```

- 参数说明:

|名称|类型|说明|
|---|---|---|
|teaching_range_per|int|示教范围（百分比式值），范围：[100, 200]，仅适用于主从臂的**主臂**：用于放大主臂的控制行程并映射给从臂|
|max_range_config|float|最大行程配置（单位：m），仅支持：0（无效值） / 0.07 / 0.1|
|teaching_friction|int|示教摩擦，范围：1..10，**值越大越灵敏**：更容易张合夹爪，所需用力更小|
|timeout|float|等待 ACK/校验超时时间（秒），默认 1.0|

- 返回值:

类型：`bool`（`True` 表示 ACK 收到且参数校验通过；`False` 表示超时或校验失败）

- 异常:

|异常|触发条件|
|---|---|
|TypeError|`teaching_range_per` 不是 `int`|
|ValueError|参数范围/枚举值不合法（如 `teaching_range_per` 不在 [100,200] 或 `max_range_config` 不是 0/0.07/0.1 等）|

- 使用示例:

```python
from pyAgxArm import create_agx_arm_config, AgxArmFactory

cfg = create_agx_arm_config(robot="piper", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
end_effector = robot.init_effector(robot.OPTIONS.EFFECTOR.AGX_GRIPPER)
robot.connect()

success = end_effector.set_gripper_teaching_pendant_param(
    teaching_range_per=100,
    max_range_config=0.07,
    teaching_friction=1,
)
if success:
    print("Gripper teaching pendant parameter set successfully")
```
