#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from time import time
import rclpy
from crow_msgs.msg import Runtime
from uuid import uuid4


class StatTimer():
    PROFILING_TOPIC = "/profile"
    DISABLED = False
    _clock = None

    @classmethod
    def init(cls):
        if cls._clock is None:
            cls._clock = rclpy.clock.ROSClock()
            cls._node = rclpy.create_node("profile_timer_" + str(uuid4()).replace("-", ""))
            cls._pub = cls._node.create_publisher(Runtime, cls.PROFILING_TOPIC, 10)

    @classmethod
    def _now(cls):
        return cls._clock.now()

    @classmethod
    def enter(cls, section_name, severity=Runtime.S_NORMAL):
        if cls.DISABLED:
            return
        time = cls._now().to_msg()
        msg = Runtime(section=section_name, stamp=time, action=Runtime.A_ENTER, severity=severity)
        cls._pub.publish(msg)

    @classmethod
    def exit(cls, section_name, severity=Runtime.S_NORMAL):
        if cls.DISABLED:
            return
        time = cls._now().to_msg()
        msg = Runtime(section=section_name, stamp=time, action=Runtime.A_EXIT, severity=severity)
        cls._pub.publish(msg)

    @classmethod
    def try_exit(cls, section_name, severity=Runtime.S_NORMAL):
        if cls.DISABLED:
            return
        time = cls._now().to_msg()
        msg = Runtime(section=section_name, stamp=time, action=Runtime.A_EXIT_IF_IN, severity=severity)
        cls._pub.publish(msg)
