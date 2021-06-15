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
DISABLING_TIME_LIMIT = 8  # seconds
TIMER_FREQ = 0.5 # seconds

def distance(entry):
    return entry[-1]

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
        qos = QoSProfile(depth=20, reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
        listener = self.create_subscription(msg_type=FilteredPose,
                                        topic=self.filter_topics[0],
                                        # we're using the lambda here to pass additional(topic) arg to the listner. Which then calls a different Publisher for relevant topic.
                                        callback=lambda pose_array_msg: self.input_filter_callback(pose_array_msg),
                                        qos_profile=qos) #the listener QoS has to be =1, "keep last only".
        self.get_logger().info('Input listener created on topic: "%s"' % self.filter_topics[0])
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

    def input_filter_callback(self, pose_array_msg):
        if not pose_array_msg.poses:
            self.get_logger().info("No poses received. Quitting early.")
            return  # no mask detections (for some reason)
        timestamp = datetime.fromtimestamp(pose_array_msg.header.stamp.sec+pose_array_msg.header.stamp.nanosec*(10**-9)).strftime('%Y-%m-%dT%H:%M:%SZ')
        for class_name, pose, size, uuid in zip(pose_array_msg.label, pose_array_msg.poses, pose_array_msg.size, pose_array_msg.uuid):
            self.process_detected_object(class_name, self._transform_cam2global([pose.position.x, pose.position.y, pose.position.z]), size.dimensions, uuid, timestamp)

    def input_action_callback(self, action_array_msg):
        if not action_array_msg.avg_class_name:
            self.get_logger().info("No action names received. Quitting early.")
            return  # no action detections (for some reason)
        action_name = action_array_msg.avg_class_name
        stop = action_array_msg.time_end
        self.crowracle.update_current_action(action_name, stop)
        if action_array_msg.done_class_name:
            action_name = action_array_msg.done_class_name
            start = action_array_msg.time_start
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
        all_detected = [x for x in all_detected if 'ad_' in x]
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
                self.crowracle.update_detected_object(x, location, size, uuid, timestamp)
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
                self.crowracle.update_detected_object(closest[0], location, size, uuid, timestamp)
            else: # add new detected object
                self.crowracle.add_detected_object(object_name, location, size, uuid, timestamp, corresponding_objects[0], self.id)
                self.id += 1
        elif len(corresponding_objects) > 0: # add new detected object
            self.crowracle.add_detected_object(object_name, location, size, uuid, timestamp, corresponding_objects[0], self.id)
            self.id += 1
        else:
            self.get_logger().info("Object {} not added - there is no corresponding template in the ontology.".format(object_name))

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