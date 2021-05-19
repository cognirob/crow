import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterType
from ros2param.api import call_get_parameters
import message_filters
from rclpy.action import ActionClient

from crow_msgs.msg import StampedString, CommandType, Runtime, StorageMsg
from trio3_ros2_interfaces.msg import RobotStatus, ObjectType
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
        qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
        self.create_subscription(msg_type=StampedString,
                                 topic=self.NLP_ACTION_TOPIC,
                                 callback=self.command_cb,
                                 qos_profile=qos)
        self.robot_action_client = ActionClient(self, PickNPlace, self.ROBOT_ACTION)
        self.get_logger().info(f"Connected to robot: {self.robot_action_client.wait_for_server()}")

        self.status = self.STATUS_IDLE
        self.create_timer(self.UPDATE_INTERVAL, self.update_cb)
        self.command_buffer = deque(maxlen=self.MAX_QUEUE_LENGTH)

        self._type_dict = {k: v for k, v in ObjectType.__dict__.items() if not k.startswith("_") and type(v) is int}
        self.marker_publisher = self.create_publisher(StorageMsg, self.STORAGE_TOPIC, qos_profile=1) #publishes the new storage command
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
                return (np.array(xyz), np.array(size), typ)
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
                    return (np.array(xyz), np.array(size), typ)
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
            if d["action"] == CommandType.PNP:
                op_name = "Pick & Place"
                target = self.processTarget(d["target"], d["target_type"])
                if target is None:
                    self.get_logger().error("Failed to issue Pick & Place action, target cannot be set!")
                    subprocess.run("ros2 param set /sentence_processor robot_failed True".split())
                    subprocess.run("ros2 param set /sentence_processor robot_done True".split())
                    continue
                location = self.processTarget(d["target"], d["target_type"])
                if target is None:
                    self.get_logger().error("Failed to issue Pick & Place action, location cannot be set!")
                    subprocess.run("ros2 param set /sentence_processor robot_failed True".split())
                    subprocess.run("ros2 param set /sentence_processor robot_done True".split())
                    continue
                self.get_logger().info(f"Target set to {target} and location is {location}.")
            elif d["action"] == CommandType.POINT:
                op_name = "Point"
                # target = self.processTarget(d["target"], d["target_type"])
                # if target is None:
                #     self.get_logger().error("Failed to issue pointing action, target cannot be set!")
                #     subprocess.run("ros2 param set /sentence_processor robot_failed True".split())
                #     subprocess.run("ros2 param set /sentence_processor robot_done True".split())
                #     continue
                # self.get_logger().info(f"Target set to {target}.")
                if self.DEBUG:
                    self.get_logger().fatal(f"Logic started in DEBUG MODE. Message not sent to the robot!")
                else:
                    # self.sendAction(target)
                    #self.push_actions(self.sendAction, target=target)
                    StatTimer.enter("pushing action into queue")
                    self.push_actions(self.sendAction, data_target=d["target"], data_target_type=d["target_type"])
                    StatTimer.exit("pushing action into queue")
            elif d["action"] == "define_storage":
                #@TODO: NLP sends message "define_storage"
                op_name = "Define new storage"
                self.push_actions(self.defineStorage, storage_name=d["storage_name"], marker_group_name=d["marker_group_name"])
            self.get_logger().info(f"Will perform {op_name}")

    def push_actions(self, comand, **kwargs):
        StatTimer.enter("pushing action into buffer", severity=Runtime.S_SINGLE_LINE)
        self.command_buffer.append((comand, kwargs))
        StatTimer.exit("pushing action into buffer")
        StatTimer.enter("setting param", severity=Runtime.S_SINGLE_LINE)
        subprocess.run("ros2 param set /sentence_processor robot_done True".split())
        StatTimer.exit("setting param")

    def prepare_command(self, data_target=None, data_target_type=None):
        target = None
        start_time = datetime.now()
        duration = datetime.now() - start_time
        #@TODO: data_target and data_target_type may be lists of candidates or as well dicts with constraints only
        while (target is None) and (duration.seconds <= self.TARGET_BUFFERING_TIME):
            target = self.processTarget(data_target, data_target_type)
            duration = datetime.now() - start_time
        if target is None: #@TODO: try another target candidate
            self.get_logger().error("Failed to issue pointing action, target cannot be set!")
            subprocess.run("ros2 param set /sentence_processor robot_failed True".split())
            subprocess.run("ros2 param set /sentence_processor robot_done True".split())
            return None
        else:
            self.get_logger().info(f"Target set to {target}.")
            return target

    def update_cb(self):
        if self.status & self.STATUS_IDLE: #replace IDLE by 90%DONE
            try:
                command, kwargs = self.command_buffer.pop()
            except IndexError as ie:  # no new commands to process
                pass  # noqa
            else:
                if 'data_target' in kwargs.keys():
                    target = self.prepare_command(**kwargs) #multiple attempts to identify target
                    if (self.status & self.STATUS_IDLE) and (target is not None):
                        try:
                            command(target)
                        except Exception as e:
                            self.get_logger().error(f"Error executing action {command} with args {str(target)}. The error was:\n{e}")
                            subprocess.run("ros2 param set /sentence_processor robot_done True".split())
                            subprocess.run("ros2 param set /sentence_processor robot_failed True".split())
                        finally:
                            self._set_status(self.STATUS_IDLE)
                elif 'storage_name' in kwargs.keys():
                    command(**kwargs)
            finally:
                pass

    def _set_status(self, status):
        self.status = status
        # print(self.status)

    def defineStorage(self, storage_name=None, marker_group_name=None):
        marker_msg = StorageMsg()
        marker_msg.group_name = marker_group_name
        marker_msg.storage_name = storage_name
        self.marker_publisher.publish(marker_msg)

    def sendAction(self, target, location=None, obj=None):
        StatTimer.enter("Sending command")
        self._set_status(self.STATUS_PROCESSING)
        goal_msg = PickNPlace.Goal()
        goal_msg.frame_id = "camera1_color_optical_frame"
        goal_msg.pick_pose = Pose()
        if target is not None:
            goal_msg.pick_pose.position.x, goal_msg.pick_pose.position.y, goal_msg.pick_pose.position.z = target[0]
            if target[1] is None:
                goal_msg.size = [0, 0, 0]
            else:
                goal_msg.size = target[1]
            # if target[2] is None:
            #     goal_msg.object.type = -1
            # else:
            #     goal_msg.object = ObjectType(type=target[2])
        if location is None:
            goal_msg.place_pose = Pose()
        else:
            pass  # TODO
        # goal_msg.size = [0.1, 0.2, 0.3]
        goal_msg.object_type = ObjectType(type=target[2])

        self._send_goal_future = self.robot_action_client.send_goal_async(goal_msg, feedback_callback=self.robot_feedback_cb)
        self._send_goal_future.add_done_callback(self.robot_response_cb)
        StatTimer.exit("Sending command")

    def robot_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            subprocess.run("ros2 param set /sentence_processor robot_failed True".split())
            subprocess.run("ros2 param set /sentence_processor robot_done True".split())
            self._set_status(self.STATUS_IDLE)
            return

        StatTimer.enter("robot action")
        self.get_logger().info('Goal accepted :)')
        self._set_status(self.STATUS_EXECUTING)

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.robot_done_cb)

    def robot_done_cb(self, future):
        StatTimer.exit("robot action")
        result = future.result().result
        self.get_logger().info(f'Action done, result: {result.done}')
        subprocess.run("ros2 param set /sentence_processor robot_done True".split())
        self._set_status(self.STATUS_IDLE)
        if not result.done:
            subprocess.run("ros2 param set /sentence_processor robot_failed True".split())

    def robot_feedback_cb(self, feedback_msg):
        self.get_logger().info('Got FB')
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {[(k, getattr(feedback, k)) for k in feedback.get_fields_and_field_types()]}')


def main():
    rclpy.init()
    try:
        cl = ControlLogic()
        rclpy.spin_once(cl)
        # for p, o in cl.onto.predicate_objects("http://imitrob.ciirc.cvut.cz/ontologies/crow#cube_holes_1"):
        #     print(p, " --- ", o)
        # time.sleep(1)
        cl.get_logger().info("ready")
        # cl.push_actions(cl.sendAction, target=None)
        # cl.push_actions(cl.sendAction, target=None)
        # cl.push_actions(cl.sendAction, target=None)
        # cl.push_actions(cl.sendAction, target=None)
        rclpy.spin(cl)
        cl.destroy_node()
    except Exception as e:
        print(f"Some error had occured: {e}")
    finally:
        subprocess.run("ros2 param set /sentence_processor robot_failed False".split())
        subprocess.run("ros2 param set /sentence_processor robot_done True".split())

if __name__ == '__main__':
    main()
