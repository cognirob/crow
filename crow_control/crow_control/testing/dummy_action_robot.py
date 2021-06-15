import asyncio
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterType
from ros2param.api import call_get_parameters
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup

from trio3_ros2_interfaces.msg import RobotStatus, CoreActionPhase, ActionResultFlag
from trio3_ros2_interfaces.srv import GetRobotStatus
from trio3_ros2_interfaces.action import PickNPlace
from rclpy.executors import MultiThreadedExecutor

from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
import time
import numpy as np


class DummyActionRobot(Node):
    ACTION_TOPICS = ["point", "pick_n_place"]  # names of actions to be created
    FAIL = False
    RANDOMLY_FAIL = True
    FAIL_OPTIONS = [ActionResultFlag.NOK_ANOTHER_PROCESS_IN_PROGRESS,
                    ActionResultFlag.NOK_GRASP_POSITION_NOT_DETECTED,
                    ActionResultFlag.NOK_COORDINATE_OUT_OF_VALID_AREA,
                    ActionResultFlag.NOK_TRAJECTORY_NOT_PLANNED,
                    ActionResultFlag.NOK_ROBOT_NOT_ACCEPTED,
                    ActionResultFlag.NOK_ROBOT_FAIL,
                    ActionResultFlag.NOK_ROBOT_NOT_FEASIBLE]
    SLEEP_TIME = 2

    def __init__(self):
        super().__init__('dummy_action_robot')
        self.actions = []
        for atp in self.ACTION_TOPICS:
            self.get_logger().info(f"Creating action server at '{atp}'")
            self.actions.append(
                ActionServer(
                    self,
                    PickNPlace,
                    atp,
                    execute_callback=self.execute_callback,
                    goal_callback=self.goal_callback,
                    cancel_callback=self.cancel_callback,
                    callback_group=ReentrantCallbackGroup()
                )
            )

    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action."""
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        self.get_logger().info(f'Executing goal with id = {goal_handle.goal_id}\n\trequest:\n{goal_handle.request}')
        # self.get_logger().info(str(goal_handle.__dict__))
        # self.get_logger().info(str(dir(goal_handle)))

        feedback_msg = PickNPlace.Feedback()
        feedback_msg.status = "status message"
        feedback_msg.core_action_phase = CoreActionPhase(phase=CoreActionPhase.ROBOTIC_ACTION)

        result = PickNPlace.Result()
        will_fail = self.FAIL or self.RANDOMLY_FAIL and np.random.rand() > 0.5
        if will_fail:
            self.get_logger().warn("The action will fail!")
        for i in range(10):
            if will_fail and i > np.random.randint(3, 7):  # FAIL
                goal_handle.abort()
                result.done = False
                result.action_result_flag = ActionResultFlag(flag=self.FAIL_OPTIONS[np.random.randint(0, len(self.FAIL_OPTIONS))])
                self.get_logger().error("Action failed!")
                return result

            if goal_handle.is_cancel_requested:  # goal has been CANCELED externally
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                result.done = False
                result.action_result_flag = ActionResultFlag(flag=0)  # TODO: set to canceled
                return result
            time.sleep(self.SLEEP_TIME)
            self.get_logger().info(f'\trunning loop {i}')
            feedback_msg.status = str((i + np.random.rand()) / 10)
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f'\t\tsent feedback')

        time.sleep(1)
        goal_handle.succeed()  # SUCCEEDED

        self.get_logger().info(f'\tDone.')
        result.done = True
        result.action_result_flag = ActionResultFlag(flag=ActionResultFlag.OK)
        return result

    def destroy(self):
        for aserver in self.actions:
            aserver.destroy()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)

    dummy_action_robot = DummyActionRobot()
    executor = MultiThreadedExecutor()
    dummy_action_robot.get_logger().info("Up!")
    rclpy.spin(dummy_action_robot, executor=executor)


if __name__ == '__main__':
    main()
