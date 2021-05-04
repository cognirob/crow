import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterType
from ros2param.api import call_get_parameters
import argparse
from crow_msgs.msg import Runtime
from rclpy.logging import get_logger
import sys
from datetime import datetime
from rclpy.time import Time


class Profiler(Node):
    PROFILING_TOPIC = "/profile"

    def __init__(self, node_name="profiler_node"):
        super().__init__(node_name)

        # parse args
        parser = argparse.ArgumentParser()
        parser.add_argument("--disable", default=None, action="store_true")
        args = parser.parse_known_args(sys.argv[1:])[0]

        enabledDesc = rclpy.node.ParameterDescriptor(type=ParameterType.PARAMETER_BOOL, description='Indicates whether profiling is enabled.')
        self.declare_parameter("enabled", value=True if args.disable is None else not args.disable, descriptor=enabledDesc)

        self.section_times = {}
        self.section_stats = {}

        self.create_subscription(Runtime, self.PROFILING_TOPIC, self.profile_cb, 10)

        # end_time = cls._now()
        # if section_name not in cls._sections_time:
        #     cls._warn(f"Trying to exit section {section_name} but it was never entered!")
        #     return

        # duration = (end_time - cls._sections_time[section_name]).nanoseconds * 1e-9
        # cls._log(f"Exiting section {section_name}, duration = {duration} seconds.")
        # del cls._sections_time[section_name]

    def profile_cb(self, msg):
        # print(dir(msg.stamp))
        stamp = Time.from_msg(msg.stamp)
        fstamp = self._format_time_msg(msg.stamp)
        logger = get_logger(msg.section)

        if msg.action == Runtime.A_ENTER:
            logger.info(f"|{fstamp}| Entering section {msg.section}.")
            self.store_start_time(stamp, msg.section)
        elif msg.action == Runtime.A_EXIT:
            duration = self.compute_duration(msg.section, stamp)
            if duration is None:
                logger.warn(f"|{fstamp}| Trying to exit section {msg.section} but it wasn't entered!")
            else:
                logger.info(f"|{fstamp}| Exiting section {msg.section}.\n\t\tDuration = {duration}")
                self.append_section_result(msg.section, duration)

    # def exit_if_in(cls, section_name):
    #     if section_name not in cls._sections_time:
    #         return
    #     end_time = cls._now()

    #     duration = (end_time - cls._sections_time[section_name]).nanoseconds * 1e-9
    #     cls._log(f"Exiting section {section_name}, duration = {duration} seconds.")
    #     del cls._sections_time[section_name]

    def store_start_time(self, time, section):
        if section in self.section_times:
            return  # TODO
        self.section_times[section] = time

    def compute_duration(self, section, end_time):
        start_time = self.pop_start_time(section)
        if start_time is None:
            return
        duration = (end_time - start_time).nanoseconds * 1e-9
        return duration

    def pop_start_time(self, section):
        if section in self.section_times:
            start_time = self.section_times[section]
            del self.section_times[section]
            return start_time

    def append_section_result(self, section_name, duration):
        if section_name not in self.section_stats:
            self.section_stats[section_name] = []
        self.section_stats[section_name].append(duration)

    @classmethod
    def print_summary(cls, *args, **kwargs):
        print("asdasfsgfsgafdgdfgafdgadfgdfg")
        cls._log(cls._sections_stat)

    @staticmethod
    def _format_time(ros_time):
        s, m = ros_time.seconds_nanoseconds()
        return datetime.fromtimestamp(s + m * 1e-9).strftime("%Y/%m/%d @ %H:%M:%S.%f")

    @staticmethod
    def _format_time_msg(time_msg):
        s, m = time_msg.sec, time_msg.nanosec
        return datetime.fromtimestamp(s + m * 1e-9).strftime("%Y/%m/%d @ %H:%M:%S.%f")

    # @classmethod
    # def _log(cls, msg):
    #     if cls.ENABLED:
    #         cls._logger.info(f"[{cls._format_time(cls._clock.now())}] {msg}")

    # @classmethod
    # def _warn(cls, msg):
    #     if cls.ENABLED:
    #         cls._logger.warn(f"[{cls._format_time(cls._clock.now())}] {msg}")

def main():
    rclpy.init()
    pf = Profiler()
    pf.get_logger().info("ready")
    rclpy.spin(pf)
    pf.destroy_node()

if __name__ == '__main__':
    main()
