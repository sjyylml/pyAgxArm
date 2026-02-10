#!/usr/bin/env python3
# -*-coding:utf8-*-

from .protocol_parser_interface import ProtocolParserInterface
from typing_extensions import Literal, Final

class DriverAPIOptions:
    class EFFECTOR:
        """
        End-effector kind constants.

        Use:
            robot.init_effector(robot.OPTIONS.EFFECTOR.AGX_GRIPPER)
        """

        AGX_GRIPPER: Final[Literal["agx_gripper"]] = "agx_gripper"
        REVO2: Final[Literal["revo2"]] = "revo2"

class DriverAPIProtoAdapter:
    pass

class ProtocolParserInterface(ProtocolParserInterface):
    
    def parse_packet(self, **kwargs):
        ...

    def pack(self, **kwargs):
        ...

    