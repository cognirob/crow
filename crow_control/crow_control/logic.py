import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterType
from ros2param.api import call_get_parameters
import message_filters

#Pointcloud
from crow_msgs.msg import StampedString, CommandType, RobotStatus
from crow_msgs.srv import GetRobotStatus
from crow_msgs.action import PickNPlace

from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from datetime import datetime
import json
import numpy as np
import cv2
import cv_bridge

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


class ControlLogic(Node):
    NLP_ACTION_TOPIC = "/nlp/command"  # processed human requests/commands

    def __init__(self, node_name="control_logic"):
        super().__init__(node_name)
        self.crowracle = CrowtologyClient(node=self)
        self.onto = self.crowracle.onto
        self.create_subscription(msg_type=StampedString,
                                 topic=self.NLP_ACTION_TOPIC,
                                 callback=self.command_cb,
                                 qos_profile=10)

    def processTarget(self, target, target_type):
        if target_type == "xyz":
            return np.array(target)
        elif target_type == "onto_id":
            try:
                uri = next(self.onto.subjects(self.crowracle.CROW.hasId, target))
            except StopIteration:
                self.get_logger().error(f"Action target was set to 'onto_id' but object with the ID '{target}' is not in the database!")
                return
            xyz = self.crowracle.get_location_of_obj(uri)
            return np.array(xyz)
        elif target_type == "onto_uri":
            try:
                xyz = self.crowracle.get_location_of_obj(uri)
            except:
                self.get_logger().error(f"Action target was set to 'onto_uri' but object '{target}' is not in the database!")
            else:
                return np.array(xyz)
        else:
            self.get_logger().error(f"Unknown action target type '{target_type}'!")

    def command_cb(self, msg):
        data = json.loads(msg.data)
        if type(data) is dict:
            data = [data]

        self.get_logger().info(f"Received {len(data)} command(s):")
        for d in data:
            if d["action"] == CommandType.PNP:
                op_name = "Pick & Place"
                target = self.processTarget(d["target"], d["target_type"])
                if target is None:
                    self.get_logger().error("Failed to issue Pick & Place action, target cannot be set!")
                    continue
                location = self.processTarget(d["target"], d["target_type"])
                if target is None:
                    self.get_logger().error("Failed to issue Pick & Place action, location cannot be set!")
                    continue
                self.get_logger().info(f"Target set to {target} and location is {location}.")
            elif d["action"] == CommandType.POINT:
                op_name = "Point"
                target = self.processTarget(d["target"], d["target_type"])
                if target is None:
                    self.get_logger().error("Failed to issue pointing action, target cannot be set!")
                    continue
                self.get_logger().info(f"Target set to {target}.")

            self.get_logger().info(f"Will perform {op_name}")
            # TODO: perform

    def sendAction(self, target, location=None, obj=None):
        pass


def main():
    rclpy.init()
    cl = ControlLogic()
    rclpy.spin(cl)
    cl.destroy_node()
    exit(0)

if __name__ == '__main__':
    main()
