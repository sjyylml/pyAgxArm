# 机械臂驱动包使用说明

[EN](README(EN).MD)

|![ubuntu18.04](https://img.shields.io/badge/Ubuntu-18.04-orange.svg)|![ubuntu20.04](https://img.shields.io/badge/Ubuntu-20.04-orange.svg)|![ubuntu22.04](https://img.shields.io/badge/Ubuntu-22.04-orange.svg)|
|---|---|---|
|![Pass](https://img.shields.io/badge/Pass-blue.svg)|![Pass](https://img.shields.io/badge/Pass-blue.svg)|![Pass](https://img.shields.io/badge/Pass-blue.svg)|

Test:

|![python3.6](https://img.shields.io/badge/Python-3.6-blue.svg)|![python3.8](https://img.shields.io/badge/Python-3.8-blue.svg)|![python3.10](https://img.shields.io/badge/Python-3.10-blue.svg)|
|---|---|---|
|![Pass](https://img.shields.io/badge/Pass-blue.svg)|![Pass](https://img.shields.io/badge/Pass-blue.svg)|![Pass](https://img.shields.io/badge/Pass-blue.svg)|

|说明 |文档|
|---|---|
|官方can模块的使用|[can_user](./docs/can_user.MD)|
|各臂接口函数的详细说明|[docs](./docs)|
|nero机械臂使用pyAgxArm读取数据|[First-TimeUserGuide(CAN)](./docs/nero/First-TimeUserGuide(CAN).MD)|
|机械臂DEMO|[demos](./pyAgxArm/demos/DEMO_README(ZH).MD)|
|CHANGELOG|[CHANGELOG](./CHANGELOG.MD)|
|Q&A|[Q&A](./docs/Q&A.MD)|

<details open>
  <summary><h2 style="display:inline">1 驱动包安装方法</h2></summary>
  <div>

## 1.1 安装依赖

注: python-can版本应高于3.3.4

```shell
pip3 install python-can
```

## 1.2 安装 pyAgxArm

[在Ubuntu24.04安装第三方pip包的方法](./docs/Ubuntu24.04安装第三方pip包的方法.MD)

注意是否在conda环境里

```bash
which pip3
```

```shell
git clone https://github.com/agilexrobotics/pyAgxArm.git
cd pyAgxArm
pip3 install .
```

## 1.3 查看 pyAgxArm 包细节

```shell
pip3 show pyAgxArm
```

## 1.4 卸载 pyAgxArm

```shell
pip3 uninstall pyAgxArm
```

## 1.5 更新 pyAgxArm

```shell
git clone https://github.com/agilexrobotics/pyAgxArm.git
cd pyAgxArm
pip3 install .
```

</div>
</details>

<details open>
  <summary><h2 style="display:inline">2 激活通讯</h2></summary>
  <div>

## 系统激活can模块

详见：[can_user](./docs/can_user.MD)

</div>
</details>

<details open>
  <summary><h2 style="display:inline">3 简单开始</h2></summary>
  <div>

## 读取关节角

```python
import time
from pyAgxArm import create_agx_arm_config, AgxArmFactory
cfg = create_agx_arm_config(robot="nero", comm="can", channel="can0")
robot = AgxArmFactory.create_arm(cfg)
# 创建通讯实例并打开读取线程
robot.connect()
while True:
    ja = robot.get_joint_angles()
    # 如果反馈数值不为None，因为刚运行程序时没有接收到数据，反馈值为None
    if ja is not None:
        print(ja.msg) # list
        print(ja.hz, ja.timestamp)
    time.sleep(0.005)# 200hz
```

</div>
</details>

## 注意事项

- 使用can协议情况下，需要先激活can设备，并且设置正确的波特率，才可以读取机械臂消息或者控制机械臂
- create_agx_arm_config 创建配置在实例化时可通过channel参数传入激活的can名称，这个名称可以通过`ifconfig`得到
- **机械臂的mit协议控制单关节电机为高级功能,使用时需注意,此协议的使用不当会导致机械臂损坏！！！**

## 联系我们

您可以在github上提issue

也可以加入discord<https://discord.gg/wrKYTxwDBd>
