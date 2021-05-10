import rclpy
from rclpy.node import Node
from ros2param.api import call_get_parameters
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from crow_msgs.msg import DetectionMask, FilteredPose, ActionDetection
from geometry_msgs.msg import PoseArray
import message_filters

import curses
import time
import numpy as np
from datetime import datetime
from curses.textpad import Textbox, rectangle
from crow_ontology.crowracle_client import CrowtologyClient
from rdflib.namespace import Namespace, RDF, RDFS, OWL, FOAF, XSD
from rdflib import URIRef, BNode, Literal, Graph
from rdflib.term import Identifier
from rcl_interfaces.srv import GetParameters

ONTO_IRI = "http://imitrob.ciirc.cvut.cz/ontologies/crow"
CROW = Namespace(f"{ONTO_IRI}#")
DELETION_TIME_LIMIT = 10  # seconds
DISABLING_TIME_LIMIT = 4  # seconds
TIMER_FREQ = 0.5 # seconds

def distance(entry):
    return entry[-1]


class OntoAdder(Node):

    def __init__(self, node_name="onto_adder"):
        super().__init__(node_name)
        self.crowracle = CrowtologyClient(node=self)
        self.onto = self.crowracle.onto
        # self.get_logger().info(self.onto)
        self.loc_threshold = 0.05 # object detected within 5cm from an older detection will be considered as the same one
        self.id = self.get_last_id() + 1
        self.ad = self.get_last_action_id() + 1

        client = self.create_client(GetParameters, f'/calibrator/get_parameters')
        if not client.wait_for_service(10):
            raise Exception("Could not get parameters from calibrator!")

        self.image_topics, self.cameras, self.camera_instrinsics, self.camera_frames = [p.string_array_value for p in call_get_parameters(node=self, node_name="/calibrator", parameter_names=["image_topics", "camera_namespaces", "camera_intrinsics", "camera_frames"]).values]
        while len(self.cameras) == 0: #wait for cams to come online
            self.get_logger().warn("No cams detected, waiting 2s.")
            time.sleep(2)
            self.image_topics, self.cameras, self.camera_instrinsics, self.camera_frames = [p.string_array_value for p in call_get_parameters(node=self, node_name="/calibrator", parameter_names=["image_topics", "camera_namespaces", "camera_intrinsics", "camera_frames"]).values]
        self.action_topics = ["action_rec"]
        self.filter_topics = ["filtered_poses"]

        #create timer for crawler - periodically delete old objects from database
        self.create_timer(TIMER_FREQ, self.timer_callback)

        #create listeners
        qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
        for i, (cam, filterTopic) in enumerate(zip(self.cameras, self.filter_topics)):
            listener = self.create_subscription(msg_type=FilteredPose,
                                          topic=filterTopic,
                                          # we're using the lambda here to pass additional(topic) arg to the listner. Which then calls a different Publisher for relevant topic.
                                          callback=lambda pose_array_msg, cam=cam: self.input_filter_callback(pose_array_msg, cam),
                                          qos_profile=qos) #the listener QoS has to be =1, "keep last only".
            self.get_logger().info('Input listener created on topic: "%s"' % filterTopic)
        listener = self.create_subscription(msg_type=ActionDetection,
                                          topic=self.action_topics[0],
                                          # we're using the lambda here to pass additional(topic) arg to the listner. Which then calls a different Publisher for relevant topic.
                                          callback=lambda action_array_msg: self.input_action_callback(action_array_msg),
                                          qos_profile=qos) #the listener QoS has to be =1, "keep last only".
        self.get_logger().info('Input listener created on topic: "%s"' % self.action_topics[0])

    def timer_callback(self):
        obj_in_database = self.crowracle.getTangibleObjects_disabled_nocls()
        now_time = datetime.now()
        for obj in obj_in_database:
            try:
                last_obj_time = next(self.onto.objects(obj, CROW.hasTimestamp))
            except StopIteration as se:
                continue
            last_obj_time = datetime.strptime(last_obj_time.toPython(), '%Y-%m-%dT%H:%M:%SZ')
            time_diff = now_time - last_obj_time
            if time_diff.seconds >= DELETION_TIME_LIMIT:
                self.crowracle.delete_object(obj)
            elif time_diff.seconds >= DISABLING_TIME_LIMIT:
                self.crowracle.disable_object(obj)
            else:
                self.crowracle.enable_object(obj)

    def input_filter_callback(self, pose_array_msg, cam):
        if not pose_array_msg.poses:
            self.get_logger().info("No poses received. Quitting early.")
            return  # no mask detections (for some reason)
        timestamp = datetime.fromtimestamp(pose_array_msg.header.stamp.sec+pose_array_msg.header.stamp.nanosec*(10**-9)).strftime('%Y-%m-%dT%H:%M:%SZ')
        for class_name, pose, size, uuid in zip(pose_array_msg.label, pose_array_msg.poses, pose_array_msg.size, pose_array_msg.uuid):
            self.process_detected_object(class_name, [pose.position.x, pose.position.y, pose.position.z], size.dimensions, uuid, timestamp)

    def input_action_callback(self, action_array_msg):
        if not action_array_msg.avg_class_name:
            self.get_logger().info("No action names received. Quitting early.")
            return  # no action detections (for some reason)
        action_name = action_array_msg.avg_class_name
        start = action_array_msg.times_start[0]
        stop = action_array_msg.times_end[-1]
        self.crowracle.add_detected_action(action_name, start, stop, self.ad)
        self.ad += 1

    def get_last_id(self):
        all_detected = list(self.onto.objects(None, CROW.hasId))
        all_detected = [int(id.split('od_')[-1]) for id in all_detected]
        num_detected = len(all_detected)
        self.get_logger().info("There are {} already detected objects in the database.".format(num_detected))
        if num_detected > 0:
            return max(all_detected)
        else:
            return -1

    def get_last_action_id(self):
        all_detected = list(self.onto.subjects(RDF.type, CROW.Action))
        all_detected = [int(str(ad).split('ad_')[-1]) for ad in all_detected]
        num_detected = len(all_detected)
        self.get_logger().info("There are {} already detected actions in the database.".format(num_detected))
        if num_detected > 0:
            return max(all_detected)
        else:
            return -1

    def process_detected_object(self, object_name, location, size, uuid, timestamp):
        if object_name in ['kuka', 'kuka_gripper']:
            #self.get_logger().info("Skipping detected object {} at location {}.".format(object_name, location))
            return
        #self.get_logger().info("Processing detected object {} at location {}.".format(object_name, location))
        prop_range = list(self.onto.objects(subject=CROW.hasDetectorName, predicate=RDFS.range))[0]
        corresponding_objects = list(self.onto.subjects(CROW.hasDetectorName, Literal(object_name, datatype=prop_range)))
        already_detected = []
        already_located = []
        for x in corresponding_objects:
            if len(list(self.onto.objects(subject=x, predicate=CROW.hasTimestamp))) > 0:
                already_detected.append(x)
        for x in already_detected:
            x_uuid = list(self.onto.objects(x, CROW.hasUuid))[0]
            if uuid == x_uuid.toPython(): # update timestamp and loc of matched already detected object
                self.crowracle.update_detected_object(x, location, size, timestamp)
                return
            else:
                old_loc_obj = list(self.onto.objects(x, CROW.hasAbsoluteLocation))[0]
                old_loc = [float(list(self.onto.objects(old_loc_obj, x))[0]) for x in [CROW.x, CROW.y, CROW.z]]
                dist = (np.linalg.norm(np.asarray(old_loc) - np.asarray(location)))
                already_located.append([x, dist])
        if len(already_located) > 0:
            already_located.sort(key=distance)
            closest = already_located[0]
            if closest[-1] <= self.loc_threshold: # update timestamp and loc of matched already detected object
                self.crowracle.update_detected_object(closest[0], location, size, timestamp)
            else: # add new detected object
                self.crowracle.add_detected_object(object_name, location, size, uuid, timestamp, corresponding_objects[0], self.id)
                self.id += 1
        elif len(corresponding_objects) > 0: # add new detected object
            self.crowracle.add_detected_object(object_name, location, size, uuid, timestamp, corresponding_objects[0], self.id)
            self.id += 1
        else:
            self.get_logger().info("Object {} not added - there is no corresponding template in the ontology.".format(object_name))

    #@TODO: assembly functions, update, move to client
    # def add_assembled_object(self, object_name, location):
    #     self.get_logger().info("Setting properties of assembled object {} at location {}.".format(object_name, location))
    #     PART = Namespace(f"{ONTO_IRI}/{object_name}#") #ns for each object (/cube_holes_1#)
    #     prop_name = PART.xyzAbsoluteLocation
    #     self.onto.set((prop_name, CROW.x, Literal(location[0], datatype=XSD.float)))
    #     self.onto.set((prop_name, CROW.y, Literal(location[1], datatype=XSD.float)))
    #     self.onto.set((prop_name, CROW.z, Literal(location[2], datatype=XSD.float)))
    #     #if enables/disables in connection:
    #     self.change_enabled(object_name, 'thread2', False) #acc to connection

    # def change_enabled(self, object_name, part_name, value):
    #     PART = Namespace(f"{ONTO_IRI}/{object_name}/{part_name}#") #ns for each object (/cube_holes_1#)
    #     OBJ = Namespace(f"{ONTO_IRI}/{object_name}#") #ns for each object (/cube_holes_1#)
    #     part_name = OBJ[part_name]
    #     prop_name = PART.enabled
    #     prop_range = list(self.onto.objects(CROW.isEnabled, RDFS.range))[0]
    #     self.onto.add((prop_name, RDF.type, prop_range))
    #     self.onto.add((prop_name, CROW.isBool, Literal(value, datatype=XSD.boolean)))
    #     self.onto.set((part_name, CROW.isEnabled, prop_name))

def main():
    rclpy.init()
    time.sleep(1)
    adder = OntoAdder()

    rclpy.spin(adder)
    adder.destroy_node()

if __name__ == "__main__":
    main()