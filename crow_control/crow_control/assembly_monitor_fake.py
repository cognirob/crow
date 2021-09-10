import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from crow_msgs.msg import AssemblyObjectProbability, AssemblyActionProbability
from rclpy.executors import MultiThreadedExecutor
from crow_ontology.utils import buildGraph
from crow_control.utils import ParamClient
from crow_ontology.crowracle_client import CrowtologyClient
from importlib.util import find_spec
import os
import numpy as np


class AssemblyMonitor(Node):
    ASSEMBLY_ACTION_TOPIC = 'assembly_action'
    ASSEMBLY_OBJECT_TOPIC = 'assembly_object'

    FAKE_MODE = "timer"  # timer or sequence

    BUILD_FILE = os.path.join(find_spec("crow_control").submodule_search_locations[0], "data", "build_snake_fixed.yaml")

    def __init__(self):
        super().__init__("assembly_fake_monitor")
        # connect to onto
        self.crowracle = CrowtologyClient(node=self)
        self.onto = self.crowracle.onto
        # get object / action dicts
        self.objects = [o[2:].lower() for o in dir(AssemblyObjectProbability) if o.startswith("O_")]
        self.actions = [a[2:].lower() for a in dir(AssemblyActionProbability) if a.startswith("A_")]

        # publishers
        self.action_pub = self.create_publisher(AssemblyActionProbability, self.ASSEMBLY_ACTION_TOPIC, 10)
        self.object_pub = self.create_publisher(AssemblyObjectProbability, self.ASSEMBLY_OBJECT_TOPIC, 10)

        if self.FAKE_MODE == "timer":
            self.create_timer(1, self.send_random_data)

    def send_actions(self, actions):
        pass

    def send_objects(self, objects):
        pass

    def generate_random_actions(self):
        r = np.random.rand(len(self.actions))
        r /= r.sum()
        return r

    def generate_random_objects(self):
        r = np.random.rand(len(self.objects))
        r /= r.sum()
        return r

    def send_random_data(self):
        if np.random.rand() > 0.5:
            aap = AssemblyActionProbability(probabilities=self.generate_random_actions())
            self.action_pub.publish(aap)
        else:
            aop = AssemblyObjectProbability(probabilities=self.generate_random_objects())
            self.object_pub.publish(aop)

    def generate_fake_data(self):
        pass

    def load_assembly(self, assembly_name: str):
        graph, recipe_name, assembly_name, base_filename = buildGraph(self.BUILD_FILE, self.onto)
        first = {}
        second = {}
        for edge in graph.edges_iter():
            if "! >" in edge[0]:
                idx = int(edge[0][3:])
                second[idx] = edge[1]
            elif "! >" in edge[1]:
                idx = int(edge[1][3:])
                first[idx] = edge[0]
        for i in range(len(first) - 1):
            f, s = first[i], second[i]
            felms = [a if f in b else b for (a, b) in [elm for elm in graph.edges(f) if not ("! >" in elm[0] or "! >" in elm[1])]]
            selms = [a if s in b else b for (a, b) in [elm for elm in graph.edges(s) if not ("! >" in elm[0] or "! >" in elm[1])]]



def main():
    rclpy.init()
    am = AssemblyMonitor()
    try:
        n_threads = 2
        mte = MultiThreadedExecutor(num_threads=n_threads, context=rclpy.get_default_context())
        rclpy.spin_once(am, executor=mte)
        am.get_logger().info("ready")
        rclpy.spin(am, executor=mte)
        am.destroy_node()
    except KeyboardInterrupt:
        am.destroy_node()
        print("User requested shutdown.")
    except BaseException as e:
        print(f"Some error had occured: {e}")

if __name__ == '__main__':
    main()
