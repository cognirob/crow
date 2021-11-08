import rclpy
from rclpy.node import Node
from ros2param.api import call_get_parameters
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from crow_msgs.msg import DetectionMask, FilteredPose, ActionDetection
from geometry_msgs.msg import PoseArray
import message_filters
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import traceback as tb
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
from crow_control.utils import ParamClient
from threading import Thread, RLock


ONTO_IRI = "http://imitrob.ciirc.cvut.cz/ontologies/crow"
CROW = Namespace(f"{ONTO_IRI}#")
DELETION_TIME_LIMIT = 10  # seconds
DISABLING_TIME_LIMIT = 3  # seconds
TIMER_FREQ = 0.4  # seconds
CLOSE_THRESHOLD = 2e-2  # 2cm
MIN_DIST_TO_UPDATE = 5e-3  # if object's position is less than this, object is not updated


def distance(entry):
    return entry[-1]


class OntoAdder(Node):
    ACTION_TOPIC = "/action_rec"
    FILTERED_POSES_TOPIC = "/filtered_poses"

    def __init__(self, node_name="onto_adder"):
        super().__init__(node_name)
        self.crowracle = CrowtologyClient(node=self)
        self.onto = self.crowracle.onto
        # self.get_logger().info(self.onto)
        self.loc_threshold = 0.05  # object detected within 5cm from an older detection will be considered as the same one
        self.id = self.get_last_id() + 1
        self.ad = self.get_last_action_id() + 1

        self._rlock = RLock()
        # self.get_logger().set_level(40)

        # client = self.create_client(GetParameters, f'/calibrator/get_parameters')
        # if not client.wait_for_service(10):
        #     raise Exception("Could not get parameters from calibrator!")

        # self.image_topics, self.cameras, self.camera_instrinsics, self.camera_frames = [p.string_array_value for p in call_get_parameters(node=self, node_name="/calibrator", parameter_names=["image_topics", "camera_namespaces", "camera_intrinsics", "camera_frames"]).values]
        # while len(self.cameras) == 0: #wait for cams to come online
        #     self.get_logger().warn("No cams detected, waiting 2s.")
        #     time.sleep(2)
        #     self.image_topics, self.cameras, self.camera_instrinsics, self.camera_frames = [p.string_array_value for p in call_get_parameters(node=self, node_name="/calibrator", parameter_names=["image_topics", "camera_namespaces", "camera_intrinsics", "camera_frames"]).values]

        # create timer for crawler - periodically delete old objects from database
        self.create_timer(TIMER_FREQ, self.timer_callback, callback_group=MutuallyExclusiveCallbackGroup())

        # create listeners
        qos = QoSProfile(depth=3, reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
        self.create_subscription(msg_type=FilteredPose,
                                 topic=self.FILTERED_POSES_TOPIC,
                                 callback=self.input_filter_callback,
                                 callback_group=MutuallyExclusiveCallbackGroup(),
                                 qos_profile=qos)
        self.get_logger().info(f'Input listener created on topic: {self.FILTERED_POSES_TOPIC}')
        self.create_subscription(msg_type=ActionDetection,
                                 topic=self.ACTION_TOPIC,
                                 callback=self.input_action_callback,
                                 callback_group=MutuallyExclusiveCallbackGroup(),
                                 qos_profile=qos)
        self.get_logger().info(f'Input listener created on topic: {self.ACTION_TOPIC}')

        self.pclient = ParamClient()
        self.pclient.define("adder_alive", True)

        # Storage
        self.storage_space_added = False
        self.crowracle.add_storage_space_flat("front_stage", [
            [1.25, 1],
            [-0.2, 1],
            [-0.2, 0.3],
            [1.25, 0.3],
        ], isMainArea=True)
        self.crowracle.add_storage_space_flat("back_stage", [
            [1.25, 0.3],
            [-0.2, 0.3],
            [-0.2, -0.4],
            [1.25, -0.4],
        ], isMainArea=True)
        self.crowracle.add_storage_space_flat("workspace", [
            [0.65, 1],
            [0.35, 1],
            [0.35, 0.6],
            [0.65, 0.6],
        ], isMainArea=True)

    def timer_callback(self):
        self.pclient.adder_alive = time.time()
        # start = time.time()
        tmsg = self.get_clock().now().to_msg()
        now_time = datetime.fromtimestamp(tmsg.sec + tmsg.nanosec * 1e-9)
        for obj, last_obj_time, enabled in self.crowracle.getTangibleObjects_timestamp():
            if last_obj_time is None:
                self.get_logger().warn(f'Trying to check timestamp of object {obj} failed. It has not timestamp!')
                continue
            last_obj_time = datetime.strptime(last_obj_time.toPython(), '%Y-%m-%dT%H:%M:%SZ')
            time_diff = (now_time - last_obj_time).total_seconds()
            if time_diff >= DELETION_TIME_LIMIT:
                self.crowracle.delete_object(obj)
            elif time_diff >= DISABLING_TIME_LIMIT:
                self.get_logger().warn(f'Disabling limit reached for object {obj}. Status: {enabled}')
                if enabled:
                    self.crowracle.disable_object(obj)
            elif not enabled:
                self.get_logger().warn(f'Trying to enable {obj}. Status: {enabled}')
                self.crowracle.enable_object(obj)

        self.crowracle.pair_objects_to_areas_wq()
        # pairing_thread = Thread(target=self.pair_objects_2_areas, daemon=True)
        # pairing_thread.start()

    def pair_objects_2_areas(self):
        try:
            self._rlock.acquire()
            self.crowracle.pair_objects_to_areas_wq()
        except BaseException as e:
            pass
        finally:
            self._rlock.release()

    def input_filter_callback(self, pose_array_msg):
        if not pose_array_msg.poses:
            self.get_logger().info("No poses received. Quitting early.")
            return
        # start = time.time()
        tmsg = self.get_clock().now()
        self.get_logger().warn(f"Time difference is {(tmsg - rclpy.time.Time.from_msg(pose_array_msg.header.stamp)).nanoseconds * 1e-9:0.4f}")
        # now_time = datetime.fromtimestamp(tmsg.sec + tmsg.nanosec * 1e-9)

        self.get_logger().error(str(pose_array_msg.tracked))
        timestamp = datetime.fromtimestamp(pose_array_msg.header.stamp.sec+pose_array_msg.header.stamp.nanosec*(10**-9)).strftime('%Y-%m-%dT%H:%M:%SZ')
        update_dict = {uuid: (class_name, [pose.position.x, pose.position.y, pose.position.z if pose.position.z > 0 else 0], size.dimensions, tracked) for class_name, pose, size, uuid, tracked in zip(pose_array_msg.label, pose_array_msg.poses, pose_array_msg.size, pose_array_msg.uuid, pose_array_msg.tracked)}
        # for class_name, pose, size, uuid, tracked in zip(pose_array_msg.label, pose_array_msg.poses, pose_array_msg.size, pose_array_msg.uuid, pose_array_msg.tracked):
        # find already existing objects by uuid
        existing_objects = self.crowracle.get_objects_by_uuid(pose_array_msg.uuid)
        # update location of existing objects
        objects_to_be_updated = []

        for obj, uuid, tracked, x, y, z in existing_objects:
            str_uuid = uuid.toPython()
            xyz = np.r_[x.toPython(), y.toPython(), z.toPython()]
            if str_uuid not in update_dict:
                self.get_logger().info(f"Skipping updating of object {obj} with uuid: {uuid}. For some reason, it is missing from the update_dict: {update_dict}")
                continue
            up = update_dict[str_uuid]
            if tracked == up[3] and np.linalg.norm(xyz - up[1]) < MIN_DIST_TO_UPDATE:
                self.get_logger().info(f"Skipping object update {obj} with uuid: {uuid}. Object unchanged.")
                continue
            self.get_logger().info(f"Updating object {obj} with uuid: {uuid}")
            # self.crowracle.update_object(obj, up[1], up[2], timestamp)  # TODO: add tracking
            objects_to_be_updated.append((obj, up[1], up[2], timestamp, up[3]))
            del update_dict[str_uuid]

        # get other objects for reference
        old = list(self.crowracle.get_other_objects_by_uuid(pose_array_msg.uuid))
        foundOld = len(old) > 0
        if foundOld:  # found object with a different uuid but same location
            old_uris, *old_xyz = zip(*old)
            old_xyz = np.array(old_xyz).astype(float).T

        # add new objects (not found by uuid)
        objects_to_be_added = []
        for uuid, (class_name, pose, size, tracked) in update_dict.items():
            pose = np.r_[pose]
            if foundOld:
                close = np.where(np.linalg.norm(old_xyz - pose, axis=1) < CLOSE_THRESHOLD)[0]
                # TODO: add class name check
                if len(close) > 0:
                    close_index = close[0]
                    obj = old_uris[close_index]
                    self.get_logger().info(f"Updating object {obj}")
                    # self.crowracle.update_object(obj, pose, size, timestamp)  # TODO: add tracking
                    objects_to_be_updated.append((obj, pose, size, timestamp, tracked))
                    continue

            # self.get_logger().info(f"Adding object with class {class_name} and uuid: {uuid}")
            # self.crowracle.add_detected_object_no_template(class_name, pose, size, uuid, timestamp, self.id, tracked)
            objects_to_be_added.append((class_name, pose, size, uuid, timestamp, self.id, tracked))
            self.id += 1

        self.crowracle.update_batch_all(objects_to_be_added, objects_to_be_updated)
        # if len(objects_to_be_updated) > 0:
        #     self.crowracle.update_object_batch(objects_to_be_updated)
        # self.get_logger().warn(f"input cb takes {time.time() - start:0.3f} seconds")

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


def main():
    rclpy.init()
    time.sleep(1)
    adder = OntoAdder()
    n_threads = 3 # 1 for timer and 1 for pose updates
    try:
        # rclpy.spin(adder)
        mte = MultiThreadedExecutor(num_threads=n_threads, context=rclpy.get_default_context())
        rclpy.spin(adder, executor=mte)
    except KeyboardInterrupt:
        adder.destroy_node()
        print("User requested shutdown.")
    except BaseException as e:
        print(f"Some error had occured: {e}")
        tb.print_exc()


if __name__ == "__main__":
    main()
