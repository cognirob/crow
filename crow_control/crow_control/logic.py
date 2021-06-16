import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterType
from ros2param.api import call_get_parameters
import message_filters
from rclpy.action import ActionClient
import asyncio
from crow_msgs.msg import StampedString, CommandType, Runtime, StorageMsg
from trio3_ros2_interfaces.msg import RobotStatus, ObjectType, CoreActionPhase, Units
from trio3_ros2_interfaces.srv import GetRobotStatus
from trio3_ros2_interfaces.action import PickNPlace
# from crow_msgs.msg import StampedString, CommandType, RobotStatus, ObjectType
# from crow_msgs.srv import GetRobotStatus
# from crow_msgs.action import PickNPlace
from geometry_msgs.msg import Pose

from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from datetime import datetime
import json
import numpy as np
from num2words import num2words
import pkg_resources
import time
#from numba import jit
#from torchvision.utils import save_image
#from datetime import datetime
import os
from crow_ontology.crowracle_client import CrowtologyClient
from rdflib.namespace import Namespace, RDF, RDFS, OWL, FOAF
from rdflib import URIRef, BNode, Literal, Graph
from rdflib.term import Identifier
import time
import subprocess
from collections import deque
from crow_control.utils.profiling import StatTimer
from crow_control.utils import ParamClient, QueueServer
from crow_nlp.nlp_crow.modules.UserInputManager import UserInputManager

# QuickNDirty solutions inc. (but temporary)
global_2_robot = np.array(
    [0.7071068, 0.7071068, 0, 0,
     -0.7071068, 0.7071068, 0, 0,
     0, 0, 1, 0.233,
     0, 0, 0, 1]
).reshape(4, 4)
robot_2_global = np.linalg.inv(global_2_robot)
realsense_2_robot = np.array(
    [6.168323755264282227e-01, 3.375786840915679932e-01, -7.110263705253601074e-01, 1.405695068359375000,
     7.858521938323974609e-01, -3.148722648620605469e-01, 5.322515964508056641e-01, -0.3209410400390625000,
     -4.420567303895950317e-02, -8.870716691017150879e-01, -4.595103561878204346e-01, 0.6574929809570312500,
     0, 0, 0, 1]
).reshape(4, 4)
#@TODO: TRANSFORM IS BE DONE IN ADDER (LOC IN DATABASE IS ALREADY IN GLOBAL)

class ControlLogic(Node):
    NLP_ACTION_TOPIC = "/nlp/command"  # processed human requests/commands
    STORAGE_TOPIC = "/new_storage"
    ROBOT_ACTION_POINT = 'point'
    ROBOT_ACTION_PNP = 'pick_n_place'
    DEBUG = False
    UPDATE_INTERVAL = 0.1
    MAX_QUEUE_LENGTH = 10
    TARGET_BUFFERING_TIME = 0.3 # seconds

    STATUS_IDLE = 1
    STATUS_PROCESSING = 2
    STATUS_EXECUTING = 4

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
        self.create_subscription(msg_type=StampedString,
                                 topic=self.NLP_ACTION_TOPIC,
                                 callback=self.command_cb,
                                 callback_group=rclpy.callback_groups.ReentrantCallbackGroup(),
                                 qos_profile=qos)
        self.robot_point_client = ActionClient(self, PickNPlace, self.ROBOT_ACTION_POINT)
        self.robot_pnp_client = ActionClient(self, PickNPlace, self.ROBOT_ACTION_PNP)
        self.get_logger().info(f"Connected to robot: {self.robot_point_client.wait_for_server()}")
        self.get_logger().info(f"Connected to robot: {self.robot_pnp_client.wait_for_server()}")

        self.status = self.STATUS_IDLE
        self.create_timer(self.UPDATE_INTERVAL, self.update_main_cb, callback_group=rclpy.callback_groups.ReentrantCallbackGroup())
        self.create_timer(self.UPDATE_INTERVAL, self.update_meanwhile_cb, callback_group=rclpy.callback_groups.ReentrantCallbackGroup())
        self.command_main_buffer = QueueServer(maxlen=self.MAX_QUEUE_LENGTH, queue_name='main')
        self.command_meanwhile_buffer = deque(maxlen=self.MAX_QUEUE_LENGTH)
        self.main_buffer_count = 1
        self._type_dict = {k: v for k, v in ObjectType.__dict__.items() if not k.startswith("_") and type(v) is int}
        self.marker_publisher = self.create_publisher(StorageMsg, self.STORAGE_TOPIC, qos_profile=1) #publishes the new storage command
        self.COMMAND_DICT = {CommandType.REM_CMD_LAST: self.remove_command_last, CommandType.REM_CMD_X: self.remove_command_x,
                                CommandType.DEFINE_STORAGE: self.defineStorage, CommandType.POINT: self.sendPointAction,
                                CommandType.PICK: self.sendPickAction, CommandType.FETCH: self.sendFetchAction, CommandType.FETCH_TO: self.sendFetchToAction,
                                CommandType.RELEASE: self.sendReleaseAction, CommandType.TIDY: self.sendTidyAction}
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
        if target_type == "xyz":
            return (np.array(target), [1e-9, 1e-9, 1e-9], ObjectType.POINT)
        elif target_type == "onto_id":
            try:
                uri = next(self.onto.subjects(self.crowracle.CROW.hasId, target))
                # uri = next(self.onto.objects(self.crowracle.CROW.hasId, target))
            except:
                try:
                    uri = next(self.onto.subjects(self.crowracle.CROW.disabledId, target))
                except StopIteration:
                    self.get_logger().error(f"Action target was set to '{target_type}' but object with the ID '{target}' is not in the database!")
                    return None
        elif target_type == "onto_uri":
            uri = target
        elif target_type == "properties":
            uri = self.crowracle.get_obj_of_properties(target_ph_cls, {'color': target_ph_color, 'absolute_location': target_ph_loc}, all=False)
        else:
            self.get_logger().error(f"Unknown action target type '{target_type}'!")
            return None

        try:
            # xyz = np.array([-0.00334, 0.00232, 0.6905])
            StatTimer.enter("onto data retrieval uri", severity=Runtime.S_MINOR)
            xyz = self.crowracle.get_location_of_obj(uri)
            size = self.crowracle.get_pcl_dimensions_of_obj(uri)
            typ = self._extract_obj_type(self.crowracle.get_world_name_from_uri(uri))
            StatTimer.exit("onto data retrieval uri")
        except:
            self.get_logger().error(f"Action target was set to '{target_type}' but object '{target}' is not in the database!")
            return None
        else:
            if ('None' not in xyz) and (None not in xyz):
                return (np.array(xyz, dtype=np.float), np.array(size, dtype=np.float), int(typ))
            else:
                return None

    def command_cb(self, msg):
        StatTimer.enter("command callback")
        # self.get_logger().info("Received command msg!")
        data = json.loads(msg.data)
        if type(data) is dict:
            data = [data]
        # for p, o in self.onto.predicate_objects("http://imitrob.ciirc.cvut.cz/ontologies/crow#cube_holes_1"):
        #     print(p, " --- ", o)

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
                target_info = self.processTarget(t, target_type)
                if target_info:
                    break
            duration = datetime.now() - start_time
        if target_info is None: # try to find another target candidate
            target_info = self.processTarget(target, 'properties', **kwargs)
        if target_info is None:
            self.get_logger().error("Failed to issue action, target cannot be set!")
            self.pclient.robot_failed = True
            self.pclient.robot_done = True
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
                if (self.status & self.STATUS_IDLE) and ((kwargs.get('target_info') or kwargs.get('location')) is not None):
                    try:
                        command(**kwargs)
                    except Exception as e:
                        self.get_logger().error(f"Error executing action {disp_name} with args {kwargs}. The error was:\n{e}")
                        self.pclient.robot_done = True
                        self.pclient.robot_failed = True
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
        self.pclient.robot_done = True
        self.pclient.robot_failed = True

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
            self.get_logger().info(f'Can not remove command {command_name}, it is not in the queue')
            self.ui.buffered_say(self.guidance_file[self.LANG]["command_not_in_queue"] + disp_name + ' ' + command_name, say=2)
        self.wait_then_talk()

    def defineStorage(self, disp_name='', storage_name=None, marker_group_name=None, **kwargs):
        marker_msg = StorageMsg()
        marker_msg.group_name = marker_group_name
        marker_msg.storage_name = storage_name
        self.ui.buffered_say(self.guidance_file[self.LANG]["performing"] + disp_name, say=2)
        self.wait_then_talk()
        self.marker_publisher.publish(marker_msg)

    def sendPointAction(self, disp_name='', target_info=None, location=None, obj=None, **kwargs): #@TODO: sendPointAction
        """Point: move to target
        """
        StatTimer.enter("Sending command")
        self.get_logger().info("Performing Point action")
        self.pclient.robot_done = False
        self._set_status(self.STATUS_PROCESSING)
        goal_msg = PickNPlace.Goal() #@TODO:
        goal_msg.frame_id = "camera1_color_optical_frame"
        goal_msg.pick_pose = Pose()
        if target_info is not None:
            goal_msg.pick_pose.position.x, goal_msg.pick_pose.position.y, goal_msg.pick_pose.position.z = target_info[0]
            if target_info[1] is None:
                goal_msg.size = [0, 0, 0]
            else:
                goal_msg.size = target_info[1]
            # if target_info[2] is None:
            #     goal_msg.object.type = -1
            # else:
            #     goal_msg.object = ObjectType(type=target_info[2])
        if location is None:
            goal_msg.place_pose = Pose()
        else:
            pass  # TODO
        # goal_msg.size = [0.1, 0.2, 0.3]
        goal_msg.object_type = ObjectType(type=target_info[2])
        self.ui.buffered_say(self.guidance_file[self.LANG]["performing"] + disp_name, say=2)
        self.wait_then_talk()
        self._send_goal_future = self.robot_point_client.send_goal_async(goal_msg, feedback_callback=self.robot_feedback_cb)
        self._send_goal_future.add_done_callback(self.robot_response_cb)
        StatTimer.exit("Sending command")

    def sendPNPAction(self, disp_name='', target_info=None, location=None, obj=None, **kwargs):
        #@TODO: not used
        StatTimer.enter("Sending command")
        self.get_logger().info("Performing PNP action")
        self.pclient.robot_done = False
        self._set_status(self.STATUS_PROCESSING)
        goal_msg = PickNPlace.Goal()
        goal_msg.frame_id = "camera1_color_optical_frame"
        goal_msg.pick_pose = Pose()
        goal_msg.place_pose = Pose()
        if target_info is not None: # perform pick
            goal_msg.pick_pose.position.x, goal_msg.pick_pose.position.y, goal_msg.pick_pose.position.z = target_info[0]
            if target_info[1] is None:
                goal_msg.size = [0, 0, 0]
            else:
                goal_msg.size = target_info[1]
            if target_info[2] is None:
                goal_msg.object_type.type = -1
            else:
                goal_msg.object_type = ObjectType(type=target_info[2])
        else:
            pass # @TODO set something, None defaults to 0.0
        if location is not None: #perform place
            goal_msg.place_pose.position.x, goal_msg.place_pose.position.y, goal_msg.place_pose.position.z = location
        else:
            pass # @TODO set something, None defaults to 0.0
        self.ui.buffered_say(self.guidance_file[self.LANG]["performing"] + disp_name, say=2)
        self.wait_then_talk()
        self._send_goal_future = self.robot_pnp_client.send_goal_async(goal_msg, feedback_callback=self.robot_feedback_cb)
        self._send_goal_future.add_done_callback(self.robot_response_cb)
        StatTimer.exit("Sending command")

    def sendPickAction(self, disp_name='', target_info=None, location=None, obj=None, **kwargs):
        """Pick : move to target, pick, move to home position
        """
        StatTimer.enter("Sending command")
        self.get_logger().info("Performing Pick action")
        if self.hands_full():
            self.ui.buffered_say(self.guidance_file[self.LANG]["hands_full"] + disp_name, say=2)
            self.wait_then_talk()
            self.pclient.robot_failed = True
            return
        self.pclient.robot_done = False
        self._set_status(self.STATUS_PROCESSING)
        goal_msg = self.composePNPMessage(
                target_xyz=target_info[0],
                target_size=target_info[1],
                target_type=target_info[2],
                location_xyz=[0.53381, 0.18881, 0.22759]  # temporary "robot default" position
            )
        self.ui.buffered_say(self.guidance_file[self.LANG]["performing"] + disp_name, say=2)
        self.wait_then_talk()
        self._send_goal_future = self.robot_pnp_client.send_goal_async(goal_msg, feedback_callback=self.robot_feedback_cb)
        self._send_goal_future.add_done_callback(self.robot_response_cb)
        StatTimer.exit("Sending command")

    def sendFetchAction(self, disp_name='', target_info=None, location=None, obj=None, **kwargs):
        """Fetch (give): move to target, pick, move to user, open gripper a bit OR
                         something is already picked, move to user, open gripper a bit
        """
        StatTimer.enter("Sending command")
        self.get_logger().info("Performing Fetch action")
        self.pclient.robot_done = False
        if target_info is None:
            if self.hands_empty():
                self.ui.buffered_say(self.guidance_file[self.LANG]["hands_empty"] + disp_name, say=2)
                self.wait_then_talk()
                self.pclient.robot_failed = True
                return
            else:
                target_info = [None]
        elif (target_info is not None) and (self.hands_full()):
            self.ui.buffered_say(self.guidance_file[self.LANG]["hands_full"] + disp_name, say=2)
            self.wait_then_talk()
            self.pclient.robot_failed = True
            return
        self._set_status(self.STATUS_PROCESSING)
        goal_msg = self.composePNPMessage(
                target_xyz=target_info[0],
                target_size=target_info[1],
                target_type=target_info[2],
                location_xyz=[0.03914, 0.56487, 0.22759]  # temporary storage location
            )
        self.ui.buffered_say(self.guidance_file[self.LANG]["performing"] + disp_name, say=2)
        self.wait_then_talk()
        self._send_goal_future = self.robot_pnp_client.send_goal_async(goal_msg, feedback_callback=self.robot_feedback_cb)
        self._send_goal_future.add_done_callback(self.robot_response_cb)
        StatTimer.exit("Sending command")

    def sendFetchToAction(self, disp_name='', target_info=None, location=None, obj=None, **kwargs):
        """FetchTo (put): move to target, pick, move to location, place OR
                         something is already picked, move to location, place
        """
        StatTimer.enter("Sending command")
        self.get_logger().info("Performing FetchTo action")
        self.pclient.robot_done = False
        if (target_info is None) and (self.hands_empty()):
            self.ui.buffered_say(self.guidance_file[self.LANG]["hands_empty"] + disp_name, say=2)
            self.wait_then_talk()
            self.pclient.robot_failed = True
            return
        elif (target_info is not None) and (self.hands_full()):
            self.ui.buffered_say(self.guidance_file[self.LANG]["hands_full"] + disp_name, say=2)
            self.wait_then_talk()
            self.pclient.robot_failed = True
            return
        self._set_status(self.STATUS_PROCESSING)
        goal_msg = self.composePNPMessage(
                target_xyz=target_info[0],
                target_size=target_info[1],
                target_type=target_info[2],
                location_xyz=location
            )
        self.ui.buffered_say(self.guidance_file[self.LANG]["performing"] + disp_name, say=2)
        self.wait_then_talk()
        self._send_goal_future = self.robot_pnp_client.send_goal_async(goal_msg, feedback_callback=self.robot_feedback_cb)
        self._send_goal_future.add_done_callback(self.robot_response_cb)
        StatTimer.exit("Sending command")

    def sendReleaseAction(self, disp_name='', target_info=None, location=None, obj=None, **kwargs):
        """Release: open gripper (if holding something), move to home position
        """
        StatTimer.enter("Sending command")
        self.get_logger().info("Performing Release action")
        if self.hands_empty():
            self.ui.buffered_say(self.guidance_file[self.LANG]["hands_empty"] + disp_name, say=2)
            self.wait_then_talk()
            self.pclient.robot_failed = True
            return
        self._set_status(self.STATUS_PROCESSING)
        goal_msg = None #@TODO: message for release gripper
        self.ui.buffered_say(self.guidance_file[self.LANG]["performing"] + disp_name, say=2)
        self.wait_then_talk()
        self._send_goal_future = self.robot_pnp_client.send_goal_async(goal_msg, feedback_callback=self.robot_feedback_cb)
        self._send_goal_future.add_done_callback(self.robot_response_cb)
        StatTimer.exit("Sending command")

    def sendTidyAction(self, disp_name='', **kwargs):
        """Tidy: find all objects, perform FetchTo (to backstage) with each of them
        """
        StatTimer.enter("Sending command")
        self.get_logger().info("Performing Tidy action")
        objs = self.crowracle.getTangibleObjects()
        while len(objs) > 0:
            self._set_status(self.STATUS_PROCESSING)
            self.pclient.robot_done = False
            target_info = self.prepare_command(target=objs[0], target_type='onto_uri')
            kwargs['target_info'] = target_info
            self.sendFetchToAction(**kwargs)
            objs = self.crowracle.getTangibleObjects()
        StatTimer.exit("Sending command")

    def composePNPMessage(self, target_xyz=None, target_size=None, target_type=None, location_xyz=None):
        goal_msg = PickNPlace.Goal()
        goal_msg.frame_id = "global"
        # goal_msg.frame_id = "camera1_color_optical_frame"  # old camera frame
        goal_msg.request_units = Units(unit_type=Units.METERS)

        goal_msg.pick_pose = Pose()
        goal_msg.place_pose = Pose()
        if target_xyz is not None:
            goal_msg.pick_pose.position.x, goal_msg.pick_pose.position.y, goal_msg.pick_pose.position.z = target_xyz
            if target_size is None:
                goal_msg.size = [0, 0, 0]
            else:
                goal_msg.size = target_size
            if target_type is None:
                goal_msg.object_type.type = -1
            else:
                goal_msg.object_type = ObjectType(type=target_type)
        else:
            pass # @TODO set something, None defaults to 0.0
        if location_xyz is not None:
            goal_msg.place_pose.position.x, goal_msg.place_pose.position.y, goal_msg.place_pose.position.z = location_xyz
        else:
            pass # @TODO set something, None defaults to 0.0
        return goal_msg

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
        self.get_logger().info('Goal accepted :)')
        self._set_status(self.STATUS_EXECUTING)

        self._get_result_future = self.goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.robot_done_cb)

    def robot_done_cb(self, future):
        StatTimer.try_exit("robot action")
        StatTimer.try_exit("speech2action", severity=Runtime.S_MAIN)
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

        if feedback.core_action_phase == CoreActionPhase.ROBOTIC_ACTION:
            StatTimer.try_exit("robot action")
            StatTimer.try_exit("speech2action", severity=Runtime.S_MAIN)

    def robot_canceling_done(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('Goal successfully canceled')
            self.make_robot_fail()
        else:
            self.get_logger().info('Goal failed to cancel')

    def _transform_cam2global(self, point, return_homogeneous=False, return_as_list=True, ravel=True, auto_transpose=True) -> list:
        if type(point) is list:
            point = np.array(point)
        if point.ndim == 1:
            point = point[:, np.newaxis]
        if auto_transpose:
            shape = point.shape
            if not 1 in shape:
                self.get_logger().warn(f"Danger!!! Sending multiple points might cause problems with auto transposition!\n{str(point)}\n.")
            if 3 in shape:  # non-homogeneous point
                xyz_dir = shape.index(3)
                if xyz_dir == 1:  # row vector, not good, needs column vector
                    point = point.T
                point = np.pad(point, ((0, 1), (0, 0)), "constant", constant_values=1)  # pad to make homogeneous
            elif 4 in shape:  # homogeneous point
                xyzw_dir = shape.index(4)
                if xyzw_dir == 1:  # row vector, not good, needs column vector
                    point = point.T
            else:
                self.get_logger().error(f"Asked to convert a point but the point has an odd shape:\n{str(point)}\n.")
                return []  # return empty list to raise an error
        global_point = robot_2_global @ realsense_2_robot @ point
        if ravel:
            global_point = global_point.ravel()
        if return_as_list:
            global_point = global_point.tolist()
        if return_homogeneous:
            return global_point
        else:
            return global_point[:3]

    def hands_full(self):
        # if check robot state - both hands full - return True
        return False

    def hands_empty(self):
        # if check robot state - both hands empty - return True
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
        mte = rclpy.executors.MultiThreadedExecutor(num_threads=n_threads, context=rclpy.get_default_context())
        rclpy.spin_once(cl, executor=mte)
        # for p, o in cl.onto.predicate_objects("http://imitrob.ciirc.cvut.cz/ontologies/crow#cube_holes_1"):
        #     print(p, " --- ", o)
        # time.sleep(1)
        cl.get_logger().info("ready")
        if True:
            cl.push_actions(command_buffer='main', action_type="fetch", action=CommandType.FETCH, target=np.r_[1.0, 2.0, 3.0], target_type="xyz", location=np.r_[1.0, 2.0, 3.0], location_type="xyz", size=np.r_[2.0, 2.0, 2.0].astype(np.float))
            # cl.push_actions(command_buffer='main', action_type="point", action=CommandType.FETCH, target=np.r_[1.0, 2.0, 3.0], target_type="xyz", location=np.r_[1.0, 2.0, 3.0], location_type="xyz", size=np.r_[2.0, 2.0, 2.0].astype(np.float))
            # cl.push_actions(command_buffer='main', action_type="pick", action=CommandType.FETCH, target=np.r_[1.0, 2.0, 3.0], target_type="xyz", location=np.r_[1.0, 2.0, 3.0], location_type="xyz", size=np.r_[2.0, 2.0, 2.0].astype(np.float))
        # rclpy.spin_once(cl, executor=mte)
        rclpy.spin(cl, executor=mte)
        cl.destroy_node()
    except Exception as e:
        print(f"Some error had occured: {e}")
    finally:
        cl.pclient.robot_failed = False
        cl.pclient.robot_done = True
        cl.pclient.ready_for_next_sentence = False

if __name__ == '__main__':
    main()
