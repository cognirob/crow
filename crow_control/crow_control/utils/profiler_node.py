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
import numpy as np
import pandas as pd
import os


class Profiler(Node):
    PROFILING_TOPIC = "/profile"
    BACKUP_PERIOD = 5
    PERIODIC_SUMMARY = False
    SEVERITY = Runtime.S_NORMAL
    OVERWRITE_ENTERED_SECTION = True  # if true, entering an already entered section will overwrite the enter time
    MINIMAL_SEVERITY = -100

    def __init__(self, node_name="profiler_node"):
        super().__init__(node_name)

        # parse args
        parser = argparse.ArgumentParser()
        parser.add_argument("--disable", default=None, action="store_true")
        args = parser.parse_known_args(sys.argv[1:])[0]

        enabledDesc = rclpy.node.ParameterDescriptor(type=ParameterType.PARAMETER_BOOL, description='Indicates whether profiling is enabled.')
        self.declare_parameter("enabled", value=True if args.disable is None else not args.disable, descriptor=enabledDesc)

        self.section_times = {}
        self.section_severities = {}
        self.section_stats = {}
        self.old_section_stats = self.section_stats.copy()  # just as a comparison for the backup
        self.data = []
        pd.set_option("precision", 4)
        self.df = None

        self.create_subscription(Runtime, self.PROFILING_TOPIC, self.profile_cb, 10)
        self.create_timer(self.BACKUP_PERIOD, self.backup_data)

        self.launch_time = self.get_clock().now()
        self.launch_stamp = self._format_time_to_filestamp(self.launch_time)

    def profile_cb(self, msg):
        stamp = Time.from_msg(msg.stamp)
        fstamp = self._format_time_msg(msg.stamp)
        logger = get_logger(msg.section)

        if msg.action == Runtime.A_ENTER:
            self.enter_section(logger, fstamp, stamp, msg.section, msg.severity)
        elif msg.action == Runtime.A_EXIT:
            duration = self.compute_duration(msg.section, stamp)
            if duration is None:
                logger.warn(f"|{fstamp}| Trying to exit section {msg.section} but it wasn't entered!")
            else:
                self.report_runtime(logger, fstamp, msg.section, duration, msg.severity)
        elif msg.action == Runtime.A_EXIT_IF_IN:
            duration = self.compute_duration(msg.section, stamp)
            if duration is not None:
                self.report_runtime(logger, fstamp, msg.section, duration, msg.severity)

    def enter_section(self, logger, fstamp, stamp, section, severity):
        logger.info(f"|{fstamp}| >>> Entering section {section}...")
        self.store_start_time(stamp, section, severity)

    def report_runtime(self, logger, fstamp, section, duration, severity):
        logger.info(f"|{fstamp}| Exiting section {section} <<<\n\t\tDuration = {duration:.04f} seconds.")
        if section not in self.section_severities or self.section_severities[section] == Runtime.S_NORMAL:
            # store severity or overwrite current severity if it is normal
            self.section_severities[section] = severity
        self.append_section_result(section, duration)

    def store_start_time(self, time, section, severity):
        if section in self.section_times and not self.OVERWRITE_ENTERED_SECTION:
            return
        if section not in self.section_severities or self.section_severities[section] == Runtime.S_NORMAL:
            # store severity or overwrite current severity if it is normal
            self.section_severities[section] = severity
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

    def backup_data(self):
        if self.section_stats == self.old_section_stats:
            return
        self.old_section_stats = self.section_stats.copy()
        self.compute_summary()
        self.store_summary()
        if self.PERIODIC_SUMMARY:
            self.print_summary()

    def compute_summary(self, create_df=True):
        self.data = []
        for section, runlist in self.section_stats.items():
            runtimes = np.array(runlist)
            self.data.append(
                {
                    "section": section,
                    "mean time": runtimes.mean(),
                    "median time": np.median(runtimes),
                    "st.dev.": u"\u00B1" + f"{runtimes.std():.5f}",
                    "# of runs": len(runlist),
                    "min": runtimes.min(),
                    "max": runtimes.max(),
                    "severity": self.section_severities[section]
                }
            )
        if create_df:
            self.convert_summary_2pd()

    def convert_summary_2pd(self):
        filtered_data = filter(lambda x: x["severity"] >= self.MINIMAL_SEVERITY, self.data)
        self.df = pd.DataFrame.from_records(filtered_data)

    def store_summary(self):
        filename = os.path.join(os.getcwd(), f"runtimes_{self.launch_stamp}.csv")
        self.get_logger().info(f"Storing runtime summary to file {filename}.")
        self.df.to_csv(filename)

    def print_summary(self):
        md = self.df.to_markdown()
        print(">>>>>>> RUNTIME SUMMARY <<<<<<<")
        print(md)

    @staticmethod
    def _format_time(ros_time):
        s, m = ros_time.seconds_nanoseconds()
        return datetime.fromtimestamp(s + m * 1e-9).strftime("%Y/%m/%d @ %H:%M:%S.%f")

    @staticmethod
    def _format_time_to_filestamp(ros_time):
        s, _ = ros_time.seconds_nanoseconds()
        return datetime.fromtimestamp(s).strftime("%Y_%m_%d_%H_%M_%S")

    @staticmethod
    def _format_time_msg(time_msg):
        s, m = time_msg.sec, time_msg.nanosec
        return datetime.fromtimestamp(s + m * 1e-9).strftime("%Y/%m/%d @ %H:%M:%S.%f")


def main():
    rclpy.init()
    pf = Profiler()
    pf.get_logger().info("ready")
    try:
        rclpy.spin(pf)
    except KeyboardInterrupt:
        pf.compute_summary()
        pf.print_summary()
        pf.store_summary()
    pf.destroy_node()

if __name__ == '__main__':
    main()
