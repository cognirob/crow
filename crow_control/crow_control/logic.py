import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterType
from ros2param.api import call_get_parameters
import message_filters
from rclpy.action import ActionClient

from crow_msgs.msg import StampedString, CommandType, Runtime, StorageMsg
from trio3_ros2_interfaces.msg import RobotStatus, ObjectType, CoreActionPhase
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

class ControlLogic(Node):
    NLP_ACTION_TOPIC = "/nlp/command"  # processed human requests/commands
    STORAGE_TOPIC = "/new_storage"
    ROBOT_ACTION = 'pick_n_place'
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

        self.pclient = ParamClient()
        self.pclient.define("robot_done", True) # If true, the robot has received a goal and completed it.
        self.pclient.define("robot_failed", False) # If true, the robot had failed to perform the requested action.
        self.pclient.define("robot_planning", False) # If true, the robot has received a goal and is currently planning a trajectory for it.
        self.pclient.define("robot_executing", False) # If true, the robot has received a goal and is currently executing it.
        self.pclient.define("ready_for_next_sentence", True) # If true, sentence processor can process and send next command

        qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
        self.create_subscription(msg_type=StampedString,
                                 topic=self.NLP_ACTION_TOPIC,
                                 callback=self.command_cb,
                                 callback_group=rclpy.callback_groups.ReentrantCallbackGroup(),
                                 qos_profile=qos)
        self.robot_action_client = ActionClient(self, PickNPlace, self.ROBOT_ACTION)
        self.get_logger().info(f"Connected to robot: {self.robot_action_client.wait_for_server()}")

        self.status = self.STATUS_IDLE
        self.create_timer(self.UPDATE_INTERVAL, self.update_main_cb, callback_group=rclpy.callback_groups.ReentrantCallbackGroup())
        self.create_timer(self.UPDATE_INTERVAL, self.update_meanwhile_cb, callback_group=rclpy.callback_groups.ReentrantCallbackGroup())
        self.command_main_buffer = QueueServer(maxlen=self.MAX_QUEUE_LENGTH, queue_name='main')
        self.command_meanwhile_buffer = deque(maxlen=self.MAX_QUEUE_LENGTH)
        self.main_buffer_count = 1
        self._type_dict = {k: v for k, v in ObjectType.__dict__.items() if not k.startswith("_") and type(v) is int}
        self.marker_publisher = self.create_publisher(StorageMsg, self.STORAGE_TOPIC, qos_profile=1) #publishes the new storage command
        self.COMMAND_DICT = {CommandType.REM_CMD_LAST: self.remove_command_last, CommandType.REM_CMD_X: self.remove_command_x,
                                CommandType.DEFINE_STORAGE: self.defineStorage, CommandType.POINT: self.sendAction, 
                                CommandType.PICK: self.sendPNPAction, CommandType.FETCH: self.sendPNPAction, CommandType.FETCH_TO: self.sendPNPAction}
        StatTimer.init()

    def _extract_obj_type(self, type_str):
        if type_str in self._type_dict:
            return self._type_dict[type_str]
        else:
            return -1

    def processTarget(self, target, target_type):
        """Processes target according to its type.

        Args:
            target (any): Data or target identifier.
            target_type (str): Type of the target. Any of ["onto_id", "onto_uri", "xyz"]

        Returns:
            tuple: (<position>, <size>, <type>) - None where not applicable.
        """
        if target_type == "xyz":
            return (np.array(target), None, ObjectType.POINT)
        elif target_type == "onto_id":
            try:
                uri = next(self.onto.subjects(self.crowracle.CROW.hasId, target))
                # uri = next(self.onto.objects(self.crowracle.CROW.hasId, target))
            except:
                try:
                    uri = next(self.onto.subjects(self.crowracle.CROW.disabledId, target))
                except StopIteration:
                    self.get_logger().error(f"Action target was set to 'onto_id' but object with the ID '{target}' is not in the database!")
                    return None

            StatTimer.enter("onto data retrieval id")
            xyz = self.crowracle.get_location_of_obj(uri)
            size = self.crowracle.get_pcl_dimensions_of_obj(uri)
            typ = self._extract_obj_type(self.crowracle.get_world_name_from_uri(uri))
            StatTimer.exit("onto data retrieval id")
            if ('None' not in xyz) and (None not in xyz):
                return (np.array(xyz, dtype=np.float), np.array(size, dtype=np.float), int(typ))
            else:
                return None
        elif target_type == "onto_uri":
            try:
                # xyz = np.array([-0.00334, 0.00232, 0.6905])
                StatTimer.enter("onto data retrieval uri", severity=Runtime.S_MINOR)
                xyz = self.crowracle.get_location_of_obj(target)
                size = self.crowracle.get_pcl_dimensions_of_obj(target)
                typ = self._extract_obj_type(self.crowracle.get_world_name_from_uri(target))
                StatTimer.exit("onto data retrieval uri")
            except:
                self.get_logger().error(f"Action target was set to 'onto_uri' but object '{target}' is not in the database!")
                return None
            else:
                if ('None' not in xyz) and (None not in xyz):
                    return (np.array(xyz, dtype=np.float), np.array(size, dtype=np.float), int(typ))
                else:
                    return None
        else:
            self.get_logger().error(f"Unknown action target type '{target_type}'!")

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
        #@TODO: target and target_type may be lists of candidates or as well dicts with constraints only
        while (target_info is None) and (duration.seconds <= self.TARGET_BUFFERING_TIME):
            target_info = self.processTarget(target, target_type)
            duration = datetime.now() - start_time
        if target_info is None: #@TODO: try another target candidate
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
                        self.get_logger().error(f"Error executing action {disp_name} with args {str(target_info)}. The error was:\n{e}")
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

    def command_error(self, **kwargs):
        self.get_logger().info("Command not implemented!")
        self.pclient.robot_done = True
        self.pclient.robot_failed = True

    def remove_command_last(self, **kwargs):
        if len(self.command_main_buffer) > 0:
            self.command_main_buffer.remove(-1)
            self.get_logger().info('Removing last command')
        else:
            self.get_logger().info('Can not remove last command, it is not in the queue anymore')

    def remove_command_x(self, command_name=None, **kwargs):
        idx = self.command_main_buffer.find_name_index(command_name)
        if idx is not None:
            self.command_main_buffer.remove(idx)
            self.get_logger().info(f'Removing command {command_name}')
        else:
            self.get_logger().info('Can not remove command {command_name}, it is not in the queue')

    def defineStorage(self, storage_name=None, marker_group_name=None, **kwargs):
        marker_msg = StorageMsg()
        marker_msg.group_name = marker_group_name
        marker_msg.storage_name = storage_name
        self.marker_publisher.publish(marker_msg)

    def sendAction(self, target_info=None, location=None, obj=None, **kwargs): #@TODO: sendPointAction
        StatTimer.enter("Sending command")
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
        self._send_goal_future = self.robot_action_client.send_goal_async(goal_msg, feedback_callback=self.robot_feedback_cb)
        self._send_goal_future.add_done_callback(self.robot_response_cb)
        StatTimer.exit("Sending command")

    def sendPNPAction(self, target_info=None, location=None, obj=None, **kwargs):
        StatTimer.enter("Sending command")
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
                goal_msg.object.type = -1
            else:
                goal_msg.object = ObjectType(type=target_info[2])
        else:
            pass # @TODO set something, None defaults to 0.0
        if location is not None: #perform place
            goal_msg.place_pose.position.x, goal_msg.place_pose.position.y, goal_msg.place_pose.position.z = location
        else:
            pass # @TODO set something, None defaults to 0.0
        self._send_goal_future = self.robot_action_client.send_goal_async(goal_msg, feedback_callback=self.robot_feedback_cb)
        self._send_goal_future.add_done_callback(self.robot_response_cb)
        StatTimer.exit("Sending command")

    def robot_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            self.pclient.robot_failed = True
            self.pclient.robot_done = True
            self._set_status(self.STATUS_IDLE)
            return

        StatTimer.exit("speech2robot", severity=Runtime.S_MAIN)
        StatTimer.enter("robot action")
        self.get_logger().info('Goal accepted :)')
        self._set_status(self.STATUS_EXECUTING)

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.robot_done_cb)

    def robot_done_cb(self, future):
        StatTimer.try_exit("robot action")
        StatTimer.try_exit("speech2action", severity=Runtime.S_MAIN)
        result = future.result().result
        self.get_logger().info(f'Action done, result: {result.done}')
        self.pclient.robot_done = True
        self._set_status(self.STATUS_IDLE)
        if not result.done:
            self.pclient.robot_failed = True

    def robot_feedback_cb(self, feedback_msg):
        self.get_logger().info('Got FB')
        if feedback_msg.core_action_phase == CoreActionPhase.ROBOTIC_ACTION:
            StatTimer.try_exit("robot action")
            StatTimer.try_exit("speech2action", severity=Runtime.S_MAIN)
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {[(k, getattr(feedback, k)) for k in feedback.get_fields_and_field_types()]}')


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
        # cl.push_actions(command_buffer='main', action_type="ukaž", action=CommandType.POINT)
        # time.sleep(2)
        # cl.push_actions(command_buffer='main', action_type="seber", action=CommandType.PNP)
        # time.sleep(3)
        # rclpy.spin_once(cl, executor=mte)

        # cl.push_actions(cl.sendAction, target_info=None)
        # cl.push_actions(cl.sendAction, target_info=None)
        # cl.push_actions(cl.sendAction, target_info=None)
        # cl.push_actions(cl.sendAction, target_info=None)
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
