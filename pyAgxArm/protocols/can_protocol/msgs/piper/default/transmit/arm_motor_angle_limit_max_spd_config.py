#!/usr/bin/env python3
# -*-coding:utf8-*-
from typing_extensions import (
    Literal,
)
from ....core.attritube_base import AttributeBase

class ArmMsgMotorAngleLimitMaxSpdSet(AttributeBase):
    '''
    transmit
    
    电机角度限制/最大速度设置指令-V2(基于V1.5-2版本后增加无效数值0x7FFF)

    CAN ID:
        0x474

    Args:
        joint_index: 关节电机序号
        max_angle_limit: 最大角度限制,单位 0.1°,0x7FFF为设定无效数值
        min_angle_limit: 最小角度限制,单位 0.1°,0x7FFF为设定无效数值
        max_joint_spd: 最大关节速度,单位 0.001rad/s,范围[0,3000],0x7FFF为设定无效数值
    
    位描述:
    
        Byte 0: 关节电机序号 uint8, 值域 1-6:1-6 代表关节驱动器序号
        Byte 1: 最大角度限制 H: int16, 单位 0.1°(基于V1.5-2版本后增加无效数值0x7FFF)
        Byte 2: 最大角度限制 L
        Byte 3: 最小角度限制 H: int16, 单位 0.1°(基于V1.5-2版本后增加无效数值0x7FFF)
        Byte 4: 最小角度限制 L
        Byte 5: 最大关节速度 H: uint16, 单位 0.001rad/s(基于V1.5-2版本后增加无效数值0x7FFF)
        Byte 6: 最大关节速度 L
    '''
    '''
    transmit
    
    Motor Angle Limits/Maximum Speed Setting Command-V2(Based on version V1.5-2 and later, the invalid value 0x7FFF is added.)

    CAN ID:
        0x474

    Args:
        joint_index: Joint motor index.
        max_angle_limit: Maximum angle limit, unit 0.1°,0x7FFF is defined as the invalid value.
        min_angle_limit: Minimum angle limit, unit 0.1°,0x7FFF is defined as the invalid value.
        max_joint_spd: Maximum joint speed, unit 0.001 rad/s,Range [0, 3000],0x7FFF is defined as the invalid value.
    
    Bit Description:

        Byte 0: Joint motor index, uint8, range 1-6.
        Byte 1: Maximum angle limit (high byte), int16, unit 0.1°.(Based on version V1.5-2 and later, the invalid value 0x7FFF is added.)
        Byte 2: Maximum angle limit (low byte).
        Byte 3: Minimum angle limit (high byte), int16, unit 0.1°.(Based on version V1.5-2 and later, the invalid value 0x7FFF is added.)
        Byte 4: Minimum angle limit (low byte).
        Byte 5: Maximum joint speed (high byte), uint16, unit 0.001 rad/s.(Based on version V1.5-2 and later, the invalid value 0x7FFF is added.)
        Byte 6: Maximum joint speed (low byte).
    '''
    _VALID_JOINT_INDEX = [1, 2, 3, 4, 5, 6]

    def __init__(self, 
                 joint_index: Literal[1, 2, 3, 4, 5, 6] = 1,
                 max_angle_limit: int = 0x7FFF,
                 min_angle_limit: int = 0x7FFF,
                 max_joint_spd: int = 0x7FFF):
        if joint_index not in self._VALID_JOINT_INDEX:
            raise ValueError(f"'joint_index' Value {joint_index} out of range {self._VALID_JOINT_INDEX}")
        # if not (0 <= max_joint_spd <= 3000 or max_joint_spd == 0x7FFF):
        #     raise ValueError(f"'max_joint_spd' Value {max_joint_spd} out of range 0-3000 or not equal to 0x7FFF")
        self.joint_index = joint_index
        self.max_angle_limit = max_angle_limit
        self.min_angle_limit = min_angle_limit
        self.max_joint_spd = max_joint_spd
