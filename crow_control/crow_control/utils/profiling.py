#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from time import time
import rclpy
from datetime import datetime

class StatTimer():
    ENABLED = True
    NAME = ">>> TIMER <<<"
    _logger = None
    _clock = None
    _time = 0
    _sections_time = {}
    _sections_stat = {}

    @classmethod
    def init(cls):
        try:
            rclpy.init()
        except:
            pass

        if cls._logger is None:
            cls._logger = rclpy.logging.get_logger(cls.NAME)
            cls._clock = rclpy.clock.ROSClock()
            # rclpy.get_default_context().on_shutdown(cls.print_summary)

    @staticmethod
    def _format_time(ros_time):
        s, m = ros_time.seconds_nanoseconds()
        return datetime.fromtimestamp(s + m * 1e-9).strftime("%Y/%m/%d @ %H:%M:%S.%f")

    @classmethod
    def _log(cls, msg):
        if cls.ENABLED:
            cls._logger.info(f"[{cls._format_time(cls._clock.now())}] {msg}")

    @classmethod
    def _warn(cls, msg):
        if cls.ENABLED:
            cls._logger.warn(f"[{cls._format_time(cls._clock.now())}] {msg}")

    @classmethod
    def _now(cls):
        return cls._clock.now()

    @classmethod
    def reset(cls, enabled=True):
        cls._time = cls._now()
        cls._log(f"Timer is reset to: {cls._format_time(cls._time)}")

    @classmethod
    def enter(cls, section_name):
        cls._log(f"Entering section {section_name}")
        cls._sections_time[section_name] = cls._now()

    @classmethod
    def exit(cls, section_name):
        end_time = cls._now()
        if section_name not in cls._sections_time:
            cls._warn(f"Trying to exit section {section_name} but it was never entered!")
            return

        duration = (end_time - cls._sections_time[section_name]).nanoseconds * 1e-9
        cls._log(f"Exiting section {section_name}, duration = {duration} seconds.")
        del cls._sections_time[section_name]

    @classmethod
    def append_section_result(cls, section_name, duration):
        if section_name not in cls._sections_stat:
            cls._sections_stat[section_name] = []
        cls._sections_stat[section_name].append(duration)

    @classmethod
    def print_summary(cls, *args, **kwargs):
        print("asdasfsgfsgafdgdfgafdgadfgdfg")
        cls._log(cls._sections_stat)