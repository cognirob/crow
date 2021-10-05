import asyncio
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterType
from ros2param.api import call_get_parameters
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup

from trio3_ros2_interfaces.msg import RobotStatus, CoreActionPhase, ActionResultFlag, GripperStatus
from trio3_ros2_interfaces.srv import GetRobotStatus
from trio3_ros2_interfaces.action import RobotAction
from rclpy.executors import MultiThreadedExecutor

from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
import time
import numpy as np


class DummyActionRobot(Node):
    ACTION_TOPICS = ["point", "pick_n_place", 'release', 'pick_n_home', 'place_n_home', 'pick_n_pass', 'pass']  # names of actions to be created
    ROBOT_SERVICE_STATUS = 'get_robot_status'
    PNP_FAIL = False
    PNP_RANDOMLY_FAIL = False  # will cause the PNP action (or any action using the PNP format) to fail 50% of the time
    # RELEASE_FAIL = False
    # RELEASE_RANDOMLY_FAIL = True  # will cause the RELEASE action (or any action using the RELEASE format) to fail 50% of the time
    GRIP_ALWAYS_FULL = False
    GRIP_RANDOMLY_FULL = False  # will result in the gripper to be closed 50% of the time
    ROBOT_ALWAYS_NOT_READY = False
    ROBOT_RANDOMLY_NOT_READY = False  # will return robot_is_ready=False 50% of the time (when asking for RobotStatus)
    FAIL_OPTIONS = [ActionResultFlag.NOK_ANOTHER_PROCESS_IN_PROGRESS,
                    ActionResultFlag.NOK_GRASP_POSITION_NOT_DETECTED,
                    ActionResultFlag.NOK_COORDINATE_OUT_OF_VALID_AREA,
                    ActionResultFlag.NOK_TRAJECTORY_NOT_PLANNED,
                    ActionResultFlag.NOK_ROBOT_NOT_ACCEPTED,
                    ActionResultFlag.NOK_ROBOT_FAIL,
                    ActionResultFlag.NOK_ROBOT_NOT_FEASIBLE]
    SLEEP_TIME = 1

    def __init__(self):
        super().__init__('dummy_action_robot')
        self.actions = []
        for atp in self.ACTION_TOPICS:
            self.get_logger().info(f"Creating action server at '{atp}'")
            self.actions.append(
                ActionServer(
                    self,
                    RobotAction,
                    atp,
                    execute_callback=self.execute_pnp_callback,
                    # goal_callback=self.goal_callback,
                    cancel_callback=self.cancel_callback,
                    # execute_callback=lambda goal_handle, interface=atp: self.execute_pnp_callback(goal_handle, interface),
                    goal_callback=lambda goal_request, interface=atp: self.goal_callback(goal_request, interface),
                    # cancel_callback=lambda goal_handle, interface=atp: self.cancel_callback(goal_handle, interface),
                    callback_group=ReentrantCallbackGroup()
                )
            )
        # define robot status service
        self.srv = self.create_service(GetRobotStatus, self.ROBOT_SERVICE_STATUS, self.get_robot_status)#, callback_group=rclpy.callback_groups.ReentrantCallbackGroup())

    def get_robot_status(self, request, response):
        grip_full = self.GRIP_ALWAYS_FULL or self.GRIP_RANDOMLY_FULL and np.random.rand() > 0.5
        robot_busy = self.ROBOT_ALWAYS_NOT_READY or self.ROBOT_RANDOMLY_NOT_READY and np.random.rand() > 0.5
        self.get_logger().info(f'Got RobotStatus request for robot {request.robot_id}, sending response.{" Robot is holding something!" if grip_full else ""}')
        if robot_busy:
            self.get_logger().warn('Robot is busy!')
        response.robot_status.robot_is_ready = not robot_busy
        response.robot_status.gripper_status = GripperStatus(status=GripperStatus.GRIPPER_CLOSED if grip_full else GripperStatus.GRIPPER_OPENED)
        return response

    def goal_callback(self, goal_request, interface="/"):
        """Accept or reject a client request to begin an action."""
        self.get_logger().info(f'Received goal request for {interface} action on robot {goal_request.robot_id}.')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle, interface="/"):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info(f'Received cancel request.')
        self.get_logger().info(f'Received cancel request for {interface} action on robot {goal_handle.robot_id}.')
        return CancelResponse.ACCEPT

    async def execute_pnp_callback(self, goal_handle, interface="/"):
        self.get_logger().info(f'Executing {interface} goal with id = {goal_handle.goal_id}\n\trequest:\n{goal_handle.request}')
        # self.get_logger().info(str(goal_handle.__dict__))
        # self.get_logger().info(str(dir(goal_handle)))

        feedback_msg = RobotAction.Feedback()
        feedback_msg.status = "status message"
        feedback_msg.core_action_phase = CoreActionPhase(phase=CoreActionPhase.ROBOTIC_ACTION)

        result = RobotAction.Result()
        will_fail = self.PNP_FAIL or self.PNP_RANDOMLY_FAIL and np.random.rand() > 0.5
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
