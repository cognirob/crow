import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterType
from ros2param.api import call_get_parameters
from rclpy.action import ActionServer
import message_filters

from crow_msgs.msg import StampedString, CommandType, RobotStatus
from crow_msgs.srv import GetRobotStatus
from crow_msgs.action import PickNPlace

from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
import time
import numpy as np


class DummyActionRobot(Node):

    def __init__(self):
        super().__init__('dummy_action_robot')
        self._action_server = ActionServer(
            self,
            PickNPlace,
            'picknplace',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        self.get_logger().info(str(goal_handle))

        feedback_msg = PickNPlace.Feedback()
        feedback_msg.status = "status message"

        for i in range(10):
            time.sleep(1)
            self.get_logger().info(f'\trunning loop {i}')
            feedback_msg.progress = (i + np.random.rand()) / 10
            status = RobotStatus()
            status.gripper_status.status = i % 2
            feedback_msg.robot_status = status
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f'\t\tsent feedback')

        time.sleep(1)
        goal_handle.succeed()

        self.get_logger().info(f'\tDone.')
        result = PickNPlace.Result()
        result.done = True
        return result

def main(args=None):
    rclpy.init(args=args)

    dummy_action_robot = DummyActionRobot()
    dummy_action_robot.get_logger().info("Up!")

    rclpy.spin(dummy_action_robot)


if __name__ == '__main__':
    main()