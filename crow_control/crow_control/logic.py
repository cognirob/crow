from typing import Union
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import asyncio
from crow_msgs.msg import StampedString, CommandType, Runtime, MarkerMsg
from trio3_ros2_interfaces.msg import RobotStatus, ObjectType, CoreActionPhase, Units, GripperStatus
from trio3_ros2_interfaces.srv import GetRobotStatus
from trio3_ros2_interfaces.action import RobotAction
from geometry_msgs.msg import Pose
import traceback as tb
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from datetime import datetime
import json
import numpy as np
from num2words import num2words
import pkg_resources
import time
import os
from crow_ontology.crowracle_client import CrowtologyClient
from rdflib.namespace import Namespace, RDF, RDFS, OWL, FOAF
from rdflib import URIRef, BNode, Literal, Graph
from rdflib.term import Identifier
import time
from collections import deque
from crow_control.utils.profiling import StatTimer
from crow_control.utils import ParamClient, QueueServer
from crow_nlp.nlp_crow.modules.UserInputManager import UserInputManager


class UsefullRobotStatus():

    def __init__(self, robot_ready, gripper_open=None, gripper_closed=None, robot_id=0) -> None:
        self.__robot_id = robot_id
        self.__robot_ready = robot_ready
        if gripper_open is not None:
            if gripper_closed is not None and not (gripper_closed ^ gripper_open):
                raise ValueError("Cannot set both gripper_closed and gripper_open to the same value!!!")
            self.__gripper_open, self.__gripper_closed = gripper_open, not gripper_open
        elif gripper_closed is not None:
            if gripper_open is not None and not (gripper_closed ^ gripper_open):
                raise ValueError("Cannot set both gripper_closed and gripper_open to the same value!!!")
            self.__gripper_open, self.__gripper_closed = not gripper_closed, gripper_closed

    @property
    def robot_id(self):
        return self.__robot_id

    @property
    def robot_ready(self):
        return self.__robot_ready

    @property
    def gripper_open(self):
        return self.__gripper_open

    @property
    def gripper_closed(self):
        return self.__gripper_closed


class ControlLogic(Node):
    NLP_ACTION_TOPIC = "/nlp/command"  # processed human requests/commands
    PLANNER_ACTION_TOPIC = "/nlp/command_planner"  # requests/commands from assembly planner
    STORAGE_TOPIC = "/new_storage"
    POSITION_TOPIC = "/new_position"
    # ROBOT_ACTIONS = {

    # }
    ROBOT_ACTION_POINT = 'point'
    ROBOT_ACTION_PNP = 'pick_n_place'
    ROBOT_ACTION_OPEN = 'release'
    ROBOT_ACTION_PICK = 'pick_n_home'
    ROBOT_ACTION_PLACE = 'place_n_home'
    ROBOT_ACTION_FETCH = 'pick_n_pass'
    ROBOT_ACTION_PASS = 'pass'
    ROBOT_SERVICE_STATUS = 'get_robot_status'
    DEBUG = False
    UPDATE_INTERVAL = 0.1
    MAX_QUEUE_LENGTH = 10
    TARGET_BUFFERING_TIME = 0.3 # seconds

    STATUS_IDLE = 1
    STATUS_PROCESSING = 2
    STATUS_EXECUTING = 4

    ROBOT_ACTION_PHASE = 0

    def __init__(self, node_name="control_logic"):
        super().__init__(node_name)
        self.crowracle = CrowtologyClient(node=self)
        self.onto = self.crowracle.onto

        self.LANG = 'cs'
        self.ui = UserInputManager(language = self.LANG)
        self.guidance_file = self.ui.load_file('guidance_dialogue.json')

        self.pclient = ParamClient()
        self.pclient.define("robot_done", True) # If true, the robot has received a goal and completed it.
        self.pclient.define("robot_failed", False) # If true, the robot had failed to perform the requested action.
        self.pclient.define("robot_planning", False) # If true, the robot has received a goal and is currently planning a trajectory for it.
        self.pclient.define("robot_executing", False) # If true, the robot has received a goal and is currently executing it.
        self.pclient.define("ready_for_next_sentence", True) # If true, sentence processor can process and send next command
        self.pclient.declare("silent_mode", 2) # Set by the user (level of the talking - 1 Silence, 2 - Standard speech, 3 - Debug mode/Full speech)
        self.pclient.define("can_start_talking", True) # If true, can start playing buffered sentences

        qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
        reentrant_group = ReentrantCallbackGroup()  # no restrictions on re-entry of callbacks
        self.create_subscription(msg_type=StampedString,
                                 topic=self.NLP_ACTION_TOPIC,
                                 callback=self.command_cb,
                                 callback_group=reentrant_group,
                                 qos_profile=qos)
        self.create_subscription(msg_type=StampedString,
                                 topic=self.PLANNER_ACTION_TOPIC,
                                 callback=self.command_planner_cb,
                                 callback_group=reentrant_group,
                                 qos_profile=qos)
        self.robot_point_client = ActionClient(self, RobotAction, self.ROBOT_ACTION_POINT)
        self.robot_pnp_client = ActionClient(self, RobotAction, self.ROBOT_ACTION_PNP)
        self.gripper_open_client = ActionClient(self, RobotAction, self.ROBOT_ACTION_OPEN)
        self.robot_pick_client = ActionClient(self, RobotAction, self.ROBOT_ACTION_PICK)
        self.robot_place_client = ActionClient(self, RobotAction, self.ROBOT_ACTION_PLACE)
        self.robot_fetch_client = ActionClient(self, RobotAction, self.ROBOT_ACTION_FETCH)
        self.robot_pass_client = ActionClient(self, RobotAction, self.ROBOT_ACTION_PASS)
        # self.get_logger().info(f"Connected to robot point action: {self.robot_point_client.wait_for_server()}")
        # self.get_logger().info(f"Connected to robot pnp action: {self.robot_pnp_client.wait_for_server()}")
        # self.get_logger().info(f"Connected to gripper open action: {self.gripper_open_client.wait_for_server()}")

        self.robot_status_client = self.create_client(GetRobotStatus, self.ROBOT_SERVICE_STATUS, callback_group=reentrant_group)
        # self.get_logger().info(f"Connected to robot status service: {self.robot_status_client.wait_for_service()}")

        self.status = self.STATUS_IDLE
        self.create_timer(self.UPDATE_INTERVAL, self.update_main_cb, callback_group=MutuallyExclusiveCallbackGroup())
        self.create_timer(self.UPDATE_INTERVAL, self.update_meanwhile_cb, callback_group=MutuallyExclusiveCallbackGroup())
        self.command_main_buffer = QueueServer(maxlen=self.MAX_QUEUE_LENGTH, queue_name='main')
        self.command_meanwhile_buffer = deque(maxlen=self.MAX_QUEUE_LENGTH)
        self.main_buffer_count = 1
        self._type_dict = {k: v for k, v in ObjectType.__dict__.items() if not k.startswith("_") and type(v) is int}
        self.marker_storage_publisher = self.create_publisher(MarkerMsg, self.STORAGE_TOPIC, qos_profile=1) #publishes the new storage command
        self.marker_position_publisher = self.create_publisher(MarkerMsg, self.POSITION_TOPIC, qos_profile=1) #publishes the new position command
        self.COMMAND_DICT = {CommandType.REM_CMD_LAST: self.remove_command_last,
                             CommandType.REM_CMD_X: self.remove_command_x,
                             CommandType.DEFINE_STORAGE: self.defineStorage,
                             CommandType.DEFINE_POSITION: self.definePosition,
                             CommandType.POINT: self.sendPointAction,
                             CommandType.PICK: self.sendPickAction,
                             CommandType.FETCH: self.sendFetchAction,
                             CommandType.FETCH_TO: self.sendFetchToAction,
                             CommandType.RELEASE: self.sendReleaseAction,
                             CommandType.TIDY: self.sendTidyAction}
        StatTimer.init()

    def _extract_obj_type(self, type_str):
        if type_str in self._type_dict:
            return self._type_dict[type_str]
        else:
            return -1

    def processTarget(self, target, target_type, target_ph_cls=None, target_ph_color=None, target_ph_loc=None, **kwargs):
        """Processes target according to its type.

        Args:
            target (any): Data or target identifier.
            target_type (str): Type of the target. Any of ["onto_id", "onto_uri", "xyz"]

        Returns:
            tuple: (<position>, <size>, <type>) - None where not applicable.
        """
        # print(target)
        # print(target_type)
        # print(target_ph_cls)
        # print(target_ph_loc)
        StatTimer.enter("onto data retrieval uri", severity=Runtime.S_MINOR)
        if target_type == "xyz":
            return (np.array(target), [1e-9, 1e-9, 1e-9], ObjectType.POINT)
        elif target_type == "onto_id":
            try:
                uri = next(self.onto.subjects(self.crowracle.CROW.hasId, target))
            except:
                self.get_logger().error(f"Action target was set to '{target_type}' but object with the ID '{target}' is not in the database!")
                return None
                try:
                    uri = next(self.onto.subjects(self.crowracle.CROW.disabledId, target))
                except StopIteration:
                    return None
        elif target_type == "onto_uri":
            uri = URIRef(target + "a")
        elif target_type == "properties":
            color = self.crowracle.get_uri_from_nlp(target_ph_color)
            uri = (self.crowracle.get_obj_of_properties(target_ph_cls, {'color': color}, all=False))[0]
            #TODO: check location - is within an area/storage?
            # location = self.crowracle.get_location_of_obj(uri)
            # dist = np.linalg.norm(np.asarray(location) - np.asarray(target_ph_loc))
            # if dist > 0.1:
            #     self.get_logger().error(f"Target location was set to '{target_ph_loc}' but object '{target_ph_class}' is not there!")
            #     return None
        else:
            self.get_logger().error(f"Unknown action target type '{target_type}'!")
            return None

        try:
            # xyz = np.array([-0.00334, 0.00232, 0.6905])
            res = self.crowracle.get_target_from_uri(uri)
            if res is None:
                res = self.crowracle.get_target_from_type(URIRef(target_ph_cls))
            xyz, size, typ = res
            typ = self._extract_obj_type(typ)
        except:
            self.get_logger().error(f"Action target was set to '{target_type}' but object '{target}' is not in the database! Trying another {target_ph_cls}")
            StatTimer.exit("onto data retrieval uri")
            return None
        else:
            StatTimer.exit("onto data retrieval uri")
            if ('None' not in xyz) and (None not in xyz):
                return (np.array(xyz, dtype=np.float), np.array(size, dtype=np.float), int(typ))
            else:
                return None

    def processLocation(self, location, location_type="xyz"):
        """Processes location according to its type.

        Args:
            location (any): Data or location identifier.
            location_type (str): Type of the location. Any of ["storage", "position", "xyz"]

        Returns:
            list: xyz position.
        """
        if location_type == "xyz":
            return np.array(location)
        elif location_type == "storage":
            return self.crowracle.get_free_space_area(location)
        elif location_type == "position":
            return self.crowracle.get_location_of_obj(location)
        else:
            self.get_logger().error(f"Unknown action location type '{location_type}'!")
            return None

    def command_cb(self, msg):
        StatTimer.enter("command callback")
        # self.get_logger().info("Received command msg!")
        data = json.loads(msg.data)
        if type(data) is dict:
            data = [data]

        self.get_logger().info(f"Received {len(data)} command(s):")
        self.get_logger().info(f"Received {data}")
        self.process_actions(data)
        StatTimer.exit("command callback")

    def command_planner_cb(self, msg):
        StatTimer.enter("command callback")
        self.get_logger().info("Received planner command msg!")
        data = json.loads(msg.data)
        if type(data) is dict:
            data = [data]
        # TODO: priority
        self.get_logger().info(f"Received {len(data)} command(s):")
        self.get_logger().info(f"Received {data}")
        self.process_actions(data)
        StatTimer.exit("command callback")

    def process_actions(self, data):
        for d in data:
            buffer = d.setdefault("command_buffer", 'main')
            op_name = d.get("action_type")
            if self.DEBUG:
                self.get_logger().fatal(f"Logic started in DEBUG MODE. Message not sent to the robot!")
            else:
                StatTimer.enter("pushing action into queue")
                self.push_actions(**d)
                StatTimer.exit("pushing action into queue")
            self.get_logger().info(f"Will perform {op_name}")

    def push_actions(self, command_buffer='main', action_type=None, action=None, **kwargs):
        StatTimer.enter("pushing action into buffer", severity=Runtime.S_SINGLE_LINE)
        if command_buffer == 'meanwhile':
            self.command_meanwhile_buffer.append((action_type, action, kwargs))
        else:
            command_name = str(self.main_buffer_count)#num2words(self.main_buffer_count, lang='cz')
            self.command_main_buffer.append((action_type, action, command_name, kwargs))
            self.main_buffer_count += 1
        StatTimer.exit("pushing action into buffer")
        StatTimer.enter("setting param", severity=Runtime.S_SINGLE_LINE)
        self.pclient.ready_for_next_sentence = True
        StatTimer.exit("setting param")

    def prepare_command(self, target=None, target_type=None, **kwargs):
        target_info = None
        start_time = datetime.now()
        duration = datetime.now() - start_time
        # target and target_type may be lists of candidates or as well dicts with constraints only
        if type(target) is not list:
            target = [target]
        while (target_info is None) and (duration.seconds <= self.TARGET_BUFFERING_TIME):
            for t in target:
                target_info = self.processTarget(t, target_type, **kwargs)
                if target_info:
                    break
            duration = datetime.now() - start_time
        if target_info is None: # try to find another target candidate
            target_info = self.processTarget(target, 'properties', **kwargs)
        if target_info is None:
            self.get_logger().error("Failed to issue action, target cannot be set!")
            self.make_robot_fail_to_start()
            return None
        else:
            self.get_logger().info(f"Target set to {target_info}.")
            return target_info

    def update_main_cb(self):
        if self.status & self.STATUS_IDLE: #replace IDLE by 90%DONE
            try:
                disp_name, action, command_name, kwargs = self.command_main_buffer.pop()
                kwargs['disp_name'] = disp_name
            except IndexError as ie:  # no new commands to process
                pass  # noqa
            else:
                command = self.COMMAND_DICT.get(action, self.command_error)
                if 'target' in kwargs.keys():
                    target_info = self.prepare_command(**kwargs) #multiple attempts to identify target
                    kwargs['target_info'] = target_info
                if 'location' in kwargs.keys():
                    # FIXME: make sure location type is sent from NLP! (this is probably missing from templates)
                    if 'location_type' not in kwargs:
                        kwargs['location_type'] = 'xyz'
                    location = self.processLocation(kwargs['location'], kwargs['location_type'])
                    print('locs', location)
                    kwargs['location'] = location
                if (self.status & self.STATUS_IDLE) and ((kwargs.get('target_info') or kwargs.get('location')) is not None):
                    try:
                        command(**kwargs)
                    except Exception as e:
                        self.get_logger().error(f"Error executing action {disp_name} with args {kwargs}. The error was:\n{e}")
                        self.make_robot_fail_to_start()
                    finally:
                        self._set_status(self.STATUS_IDLE)
                else:
                    command(**kwargs)
            finally:
                pass

    def update_meanwhile_cb(self):
        try:
            disp_name, action, kwargs = self.command_meanwhile_buffer.pop()
            kwargs['disp_name'] = disp_name
        except IndexError as ie:  # no new commands to process
            pass  # noqa
        else:
            command = self.COMMAND_DICT.get(action, self.command_error)
            command(**kwargs)
        finally:
                pass

    def _set_status(self, status):
        self.status = status
        # print(self.status)

    def command_error(self, disp_name='', **kwargs):
        self.get_logger().info("Command not implemented!")
        self.ui.buffered_say(disp_name + self.guidance_file[self.LANG]["template_non_implemented"], say=2)
        self.wait_then_talk()
        self.make_robot_fail_to_start()

    def remove_command_last(self, disp_name='', **kwargs):
        if len(self.command_main_buffer) > 0:
            self.command_main_buffer.remove(-1)
            self.get_logger().info('Removing last command')
            self.ui.buffered_say(self.guidance_file[self.LANG]["performing"] + disp_name, say=2)
        else:
            self.get_logger().info('Can not remove last command, it is not in the queue anymore')
            self.ui.buffered_say(self.guidance_file[self.LANG]["command_not_in_queue"] + disp_name, say=2)
        self.wait_then_talk()

    def remove_command_x(self, disp_name='', command_name=None, **kwargs):
        idx = self.command_main_buffer.find_name_index(command_name)
        if idx is not None:
            self.command_main_buffer.remove(idx)
            self.get_logger().info(f'Removing command {command_name}')
            self.ui.buffered_say(self.guidance_file[self.LANG]["performing"] + disp_name + ' ' + command_name, say=2)
        else:
            self.get_logger().info(f'Cannot remove command {command_name}, it is not in the queue')
            self.ui.buffered_say(self.guidance_file[self.LANG]["command_not_in_queue"] + disp_name + ' ' + command_name, say=2)
        self.wait_then_talk()

    def defineStorage(self, disp_name='', define_name=None, marker_group_name=None, **kwargs):
        """Marker Detector detects chosen markers, if successful, adds named storage to database
        """
        self.get_logger().info("Performing Define storage action")
        self.pclient.robot_done = False
        marker_msg = MarkerMsg()
        marker_msg.group_name = marker_group_name
        marker_msg.define_name = define_name
        self.marker_storage_publisher.publish(marker_msg)
        self.ui.buffered_say(self.guidance_file[self.LANG]["performing"] + disp_name, say=2)
        self.wait_then_talk()

    def definePosition(self, disp_name='', define_name=None, marker_group_name=None, **kwargs):
        """Marker Detector detects chosen markers, if successful, adds named position to database
        """
        self.get_logger().info("Performing Define position action")
        self.pclient.robot_done = False
        marker_msg = MarkerMsg()
        marker_msg.group_name = marker_group_name
        marker_msg.define_name = define_name
        self.marker_position_publisher.publish(marker_msg)
        self.ui.buffered_say(self.guidance_file[self.LANG]["performing"] + disp_name, say=2)
        self.wait_then_talk()
        self.get_logger().debug("Publishig marker: {}".format(marker_msg))

    def sendPointAction(self, disp_name='', target_info=None, location=None, obj=None, **kwargs): #@TODO: sendPointAction
        """Point: move to target
        """
        StatTimer.enter("Sending command")
        self.get_logger().info("Performing Point action")
        self.pclient.robot_done = False
        self._set_status(self.STATUS_PROCESSING)
        print(target_info)
        size = target_info[1]
        if np.any(np.isnan(size)):
            size = [0.0, 0.0, 0.0]
        goal_msg = self.composeRobotActionMessage(
                target_xyz=target_info[0],
                # target_size=target_info[1],
                target_size=size,
                target_type=target_info[2],
                location_xyz=location
            )
        # print('>>>>')
        self._send_goal_future = self.robot_point_client.send_goal_async(goal_msg, feedback_callback=self.robot_feedback_cb)
        self._send_goal_future.add_done_callback(self.robot_response_cb)
        self.ui.buffered_say(self.guidance_file[self.LANG]["performing"] + disp_name, say=2)
        self.wait_then_talk()
        StatTimer.exit("Sending command")

    def sendPickAction(self, disp_name='', target_info=None, location=None, obj=None, **kwargs):
        """Pick : move to target, pick, move to home position
        """
        StatTimer.enter("Sending command")
        self.get_logger().info("Performing Pick action")
        if self.hands_full():
            self.ui.buffered_say(self.guidance_file[self.LANG]["hands_full"] + disp_name, say=2)
            self.wait_then_talk()
            self.make_robot_fail_to_start()
            return
        self.pclient.robot_done = False
        self._set_status(self.STATUS_PROCESSING)
        goal_msg = self.composeRobotActionMessage(
                target_xyz=target_info[0],
                target_size=target_info[1],
                target_type=target_info[2],
                # location_xyz=location  # temporary "robot default" position - in PickTask.py template
            )
        self._send_goal_future = self.robot_pick_client.send_goal_async(goal_msg, feedback_callback=self.robot_feedback_cb)
        self._send_goal_future.add_done_callback(self.robot_response_cb)
        self.ui.buffered_say(self.guidance_file[self.LANG]["performing"] + disp_name, say=2)
        self.wait_then_talk()
        StatTimer.exit("Sending command")

    def sendFetchAction(self, disp_name='', target_info=None, location=None, obj=None, **kwargs):
        """Fetch (give): move to target, pick, move to user, open gripper a bit OR
                         something is already picked, move to user, open gripper a bit
        """
        # print(kwargs)
        StatTimer.enter("Sending command")
        self.get_logger().info("Performing Fetch action")
        self.pclient.robot_done = False
        pass_only = target_info is None
        if pass_only:
            if self.hands_empty():
                self.ui.buffered_say(self.guidance_file[self.LANG]["hands_empty"] + disp_name, say=2)
                self.wait_then_talk()
                self.make_robot_fail_to_start()
                return
            goal_msg = self.composeRobotActionMessage(
                location_xyz=location,  # temporary storage location - in Fetch.py template
                robot_id=0
            )
            client = self.robot_pass_client
        else:
            if self.hands_full():
                self.ui.buffered_say(self.guidance_file[self.LANG]["hands_full"] + disp_name, say=2)
                self.wait_then_talk()
                self.make_robot_fail_to_start()
                return
            goal_msg = self.composeRobotActionMessage(
                    target_xyz=target_info[0],
                    target_size=target_info[1],
                    target_type=target_info[2],
                    location_xyz=location  # temporary storage location - in Fetch.py template
                )
            client = self.robot_fetch_client

        self._set_status(self.STATUS_PROCESSING)
        self._send_goal_future = client.send_goal_async(goal_msg, feedback_callback=self.robot_feedback_cb)
        self._send_goal_future.add_done_callback(self.robot_response_cb)
        self.ui.buffered_say(self.guidance_file[self.LANG]["performing"] + disp_name, say=2)
        self.wait_then_talk()
        StatTimer.exit("Sending command")

    def sendFetchToAction(self, disp_name='', target_info=None, location=None, obj=None, **kwargs):
        """FetchTo (put): move to target, pick, move to location, wait for 'touch', release gripper, go home OR
                         something is already picked, move to location, wait for 'touch', release gripper, go home
        """
        StatTimer.enter("Sending command")
        self.get_logger().info("Performing FetchTo action")
        self.pclient.robot_done = False
        pass_only = target_info is None
        if pass_only:
            if self.hands_empty():
                self.ui.buffered_say(self.guidance_file[self.LANG]["hands_empty"] + disp_name, say=2)
                self.wait_then_talk()
                self.make_robot_fail_to_start()
                return
            target_info = [None]*3
            client = self.robot_place_client
            goal_msg = self.composeRobotActionMessage(
                    location_xyz=location,
                    robot_id=0
                )
        else:
            if self.hands_full():
                self.ui.buffered_say(self.guidance_file[self.LANG]["hands_full"] + disp_name, say=2)
                self.wait_then_talk()
                self.make_robot_fail_to_start()
                return
            client = self.robot_pnp_client
            goal_msg = self.composeRobotActionMessage(
                    target_xyz=target_info[0],
                    target_size=target_info[1],
                    target_type=target_info[2],
                    location_xyz=location
                )
        self._set_status(self.STATUS_PROCESSING)
        self._send_goal_future = client.send_goal_async(goal_msg, feedback_callback=self.robot_feedback_cb)
        self._send_goal_future.add_done_callback(self.robot_response_cb)
        self.ui.buffered_say(self.guidance_file[self.LANG]["performing"] + disp_name, say=2)
        self.wait_then_talk()
        StatTimer.exit("Sending command")

        # wait for 'touch'
        # self.sendReleaseAction()
        # self.sendPointAction(location=[0.53381, 0.18881, 0.22759])  # temporary "robot default" position)

    def sendReleaseAction(self, disp_name='', target_info=None, location=None, obj=None, **kwargs):
        """Release: open gripper (if holding something), move to home position
        """
        StatTimer.enter("Sending command")
        self.get_logger().info("Performing Release action")
        if self.hands_empty():
            self.ui.buffered_say(self.guidance_file[self.LANG]["hands_empty"] + disp_name, say=2)
            self.wait_then_talk()
            self.make_robot_fail_to_start()
            return
        goal_msg = self.composeRobotActionMessage(robot_id=0)
        self._send_goal_future = self.gripper_open_client.send_goal_async(goal_msg, feedback_callback=self.robot_feedback_cb)
        self._send_goal_future.add_done_callback(self.robot_response_cb)
        self.ui.buffered_say(self.guidance_file[self.LANG]["performing"] + disp_name, say=2)
        self.wait_then_talk()
        StatTimer.exit("Sending command")

    def sendTidyAction(self, disp_name='', **kwargs):
        """Tidy: find all objects, perform FetchTo (to backstage) with each of them
        """
        self.get_logger().info("Performing Tidy action")
        try:
            objs, x, y, z, dx, dy, dz, obj_type = self.crowracle.get_objects_with_poses_from_area("front_stage")
            self.get_logger().info(f"objects: {objs} @ {x}, {y}, {z} with dims {dx}, {dy}, {dz} of type {obj_type}")
            while objs is not None:
                # print(f'looping in tidy objects: STATUS {self.status} {self.STATUS_IDLE}')
                if (self.status & self.STATUS_IDLE):
                    print('ready to work in tidy objects')

                    self._set_status(self.STATUS_PROCESSING)
                    self.pclient.robot_done = False
                    print(f'Tidying object {objs} @ {x}, {y}, {z}')


                    #TODO objs list is out of date when the action is sent. The objects are newly detected and get higher index.
                    # print(f"selected object: {objs[0]}")
                    # target_info = self.prepare_command(target=objs[0], target_type='onto_uri')
                    # kwargs['target_info'] = target_info
                    self.sendFetchToAction(target_info=[[x, y, z],[dx, dy, dz], obj_type], location=[0.400, 0.065, 0.0])
                    objs, x, y, z, dx, dy, dz, obj_type = self.crowracle.get_objects_with_poses_from_area("front_stage")
                    #TODO: keep only objs in the workspace area (not in the storage, etc.)
                    # self._set_status(self.STATUS_IDLE)

        except BaseException as e:
            print(f"Error while trying to tidy stuff up: {e}")
            tb.print_exc()
        else:
            self.get_logger().info("No more objects to tidy up.")

    def composeRobotActionMessage(self, target_xyz=None, target_size=None, target_type=None, location_xyz=None, robot_id=-1):
        goal_msg = RobotAction.Goal()
        goal_msg.frame_id = "global"
        goal_msg.robot_id = robot_id
        goal_msg.request_units = Units(unit_type=Units.METERS)

        goal_msg.poses = []
        if target_xyz is not None:
            pick_pose = Pose()
            pick_pose.position.x, pick_pose.position.y, pick_pose.position.z = target_xyz
            goal_msg.poses.append(pick_pose)
            if target_size is None:  # or target_type==-1:
                goal_msg.size = [0.0, 0.0, 0.0]
            else:
                goal_msg.size = target_size
            if target_type is None:
                goal_msg.object_type.type = -1
            else:
                goal_msg.object_type = ObjectType(type=target_type)
        else:
            pass # @TODO set something, None defaults to 0.0
        print(np.isnan(goal_msg.size[0]))
        if np.isnan(goal_msg.size[0]):
            goal_msg.size = [0, 0, 0]
        if location_xyz is not None:
            place_pose = Pose()
            place_pose.position.x, place_pose.position.y, place_pose.position.z = location_xyz
            goal_msg.poses.append(place_pose)
        else:
            pass # @TODO set something, None defaults to 0.0
        return goal_msg

    def get_robot_status(self, robot_id=0) -> UsefullRobotStatus:
        future = self.robot_status_client.call_async(GetRobotStatus.Request(robot_id=robot_id))
        response = None
        while not future.done():
            time.sleep(0.1)
        try:
            res = future.result()
        except BaseException as e:
            self.get_logger().error(f"GetRobotStatus service fail: {e}")
        else:
            print(res.robot_status.gripper_status)
            response = UsefullRobotStatus(robot_id=robot_id, robot_ready=res.robot_status.robot_is_ready,
                                            gripper_closed=res.robot_status.gripper_status.status == GripperStatus.GRIPPER_CLOSED)
        return response

    def cancel_current_goal(self, wait=False):
        """Requests cancellation of the current goal.

        Returns:
            bool: Returns True, if goal can be canceled
        """
        self.get_logger().info("Trying to cancel the current goal.")
        if self.goal_handle is None:
            return False
        # self.get_logger().info(str(dir(self.goal_handle)))
        if wait:
            response = self.goal_handle.cancel_goal()
            print(response)
        else:
            future = self.goal_handle.cancel_goal_async()
            future.add_done_callback(self.robot_canceling_done)

        return True

    def robot_response_cb(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.make_robot_fail_to_start()
            return

        StatTimer.exit("speech2robot", severity=Runtime.S_MAIN)
        StatTimer.enter("robot action")
        self.ROBOT_ACTION_PHASE = 0
        StatTimer.enter("phase 0")
        self.get_logger().info('Goal accepted :)')
        self._set_status(self.STATUS_EXECUTING)

        self._get_result_future = self.goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.robot_done_cb)

    def robot_done_cb(self, future):
        StatTimer.try_exit("robot action")
        StatTimer.try_exit("speech2action", severity=Runtime.S_MAIN)
        StatTimer.enter(f"phase {str(self.ROBOT_ACTION_PHASE)}")
        result = future.result().result
        self.get_logger().info(f'Action done, result: {result.done}')
        if result.done:
            self.make_robot_done()
        else:
            self.get_logger().info(f'Action failed because: {result.action_result_flag}')
            self.make_robot_fail()

    def robot_feedback_cb(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback.status}')
        # self.get_logger().info(f'Received feedback: {feedback_msg}')
        # self.get_logger().info(f'Received feedback: {[(k, getattr(feedback, k)) for k in feedback.get_fields_and_field_types()]}')
        try:
            sn = float(feedback.status)
        except ValueError:
            pass  # noqa
        # else:
        #     if sn > 0.7:
        #         print(dir(self._get_result_future))
        #         print(self.cancel_current_goal())
        phase = feedback.core_action_phase.phase
        if phase != self.ROBOT_ACTION_PHASE:
            StatTimer.exit(f"phase {str(self.ROBOT_ACTION_PHASE)}")
            StatTimer.enter(f"phase {str(phase)}")
            self.ROBOT_ACTION_PHASE = phase
        if phase == CoreActionPhase.ROBOTIC_ACTION:
            StatTimer.try_exit("robot action")
            StatTimer.try_exit("speech2action", severity=Runtime.S_MAIN)

    def robot_canceling_done(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('Goal successfully canceled')
            self.make_robot_fail()
        else:
            self.get_logger().info('Goal failed to cancel')

    def hands_full(self):
        status_0 = self.get_robot_status(robot_id=0)
        status_1 = self.get_robot_status(robot_id=1)
        if status_0.gripper_closed and status_1.gripper_closed:
            return True
        else:
            return False

    def hands_empty(self):
        status_0 = self.get_robot_status(robot_id=0)
        status_1 = self.get_robot_status(robot_id=1)
        if status_0.gripper_open and status_1.gripper_open:
            return True
        else:
            return False

    def wait_then_talk(self):
        if self.pclient.silent_mode > 1:
            while self.pclient.can_start_talking == False:
                time.sleep(0.2)
            self.pclient.can_start_talking = False
        self.ui.buffered_say(flush=True, level=self.pclient.silent_mode)
        self.pclient.can_start_talking = True

    def make_robot_done(self):
        """Call this when robot successfully completed the last task
        """
        self._cleanup_after_action()
        self.pclient.robot_failed = False

    def make_robot_fail(self):
        """Call this when robot fails to complete an action
        """
        self._cleanup_after_action()
        self.pclient.robot_failed = True

    def make_robot_fail_to_start(self):
        """Call this when robot fails to start an action (so far, the same as failing to complete)
        """
        self.get_logger().info('Goal rejected :(')
        self._cleanup_after_action()
        self.pclient.robot_failed = True

    def _cleanup_after_action(self):
        self.goal_handle = None
        self.pclient.robot_done = True
        self._set_status(self.STATUS_IDLE)

def main():
    rclpy.init()
    cl = ControlLogic()
    try:
        n_threads = 4 # nr of callbacks in group, +1 as backup
        mte = MultiThreadedExecutor(num_threads=n_threads, context=rclpy.get_default_context())
        rclpy.spin_once(cl, executor=mte)
        # for p, o in cl.onto.predicate_objects("http://imitrob.ciirc.cvut.cz/ontologies/crow#cube_holes_1"):
        #     print(p, " --- ", o)
        # time.sleep(1)
        cl.get_logger().info("ready")
        if False:
            # cl.push_actions(command_buffer='main', action_type="fetch", action=CommandType.FETCH, target=np.r_[1.0, 2.0, 3.0], target_type="xyz", location=np.r_[1.0, 2.0, 3.0], location_type="xyz", size=np.r_[2.0, 2.0, 2.0].astype(np.float))
            cl.definePosition("modré uložiště", "modrá pozice", "modrá")
            # cl.push_actions(command_buffer='main', action_type="point", action=CommandType.FETCH, target=np.r_[1.0, 2.0, 3.0], target_type="xyz", location=np.r_[1.0, 2.0, 3.0], location_type="xyz", size=np.r_[2.0, 2.0, 2.0].astype(np.float))
            # cl.push_actions(command_buffer='main', action_type="pick", action=CommandType.FETCH, target=np.r_[1.0, 2.0, 3.0], target_type="xyz", location=np.r_[1.0, 2.0, 3.0], location_type="xyz", size=np.r_[2.0, 2.0, 2.0].astype(np.float))
        # rclpy.spin_once(cl, executor=mte)
        rclpy.spin(cl, executor=mte)
        cl.destroy_node()
    except KeyboardInterrupt:
        cl.destroy_node()
        print("User requested shutdown.")
    except BaseException as e:
        print(f"Some error had occured: {e}")
        tb.print_exc()
    finally:
        cl.pclient.robot_failed = False
        cl.pclient.robot_done = True
        cl.pclient.ready_for_next_sentence = False

if __name__ == '__main__':
    main()
