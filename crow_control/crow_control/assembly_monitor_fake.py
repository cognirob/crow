from typing import Dict, List
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from crow_msgs.msg import AssemblyObjectProbability, AssemblyActionProbability
from rclpy.executors import MultiThreadedExecutor
from crow_ontology.utils import OntoBuilder
from crow_control.utils import ParamClient
from crow_ontology.crowracle_client import CrowtologyClient
from importlib.util import find_spec
import os
import numpy as np
import traceback as tb
import sys
import time


class AssemblyMonitor(Node):
    ASSEMBLY_ACTION_TOPIC = 'assembly_action'
    ASSEMBLY_OBJECT_TOPIC = 'assembly_object'
    build_assembly = "snake"

    # FAKE_MODE = "timer"  # timer or sequence
    FAKE_MODE = "sequence"  # timer or sequence
    NOISE_LEVEL = 0.02
    TIMER_DELAY = 1

    ASSEMBLY = 'snake'
    if ASSEMBLY=='snake':
        BUILD_FILE = os.path.join(find_spec("crow_control").submodule_search_locations[0], "data", "build_snake_fixed.yaml")
    elif ASSEMBLY == 'dog':
        BUILD_FILE = os.path.join(find_spec("crow_control").submodule_search_locations[0], "data", "build_dog.yaml")
    elif ASSEMBLY == 'car':
        BUILD_FILE = os.path.join(find_spec("crow_control").submodule_search_locations[0], "data", "build_car.yaml")

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
        print(self.actions)
        print(self.objects)
        # time.sleep(5)

        if self.FAKE_MODE == "timer":
            self.create_timer(1, self.send_random_data)
        elif self.FAKE_MODE == "sequence":
            self.builder = OntoBuilder(self.onto)
            self.components, self.acts = self.load_assembly()
            self.idx = 0
            self.idx_mode = False
            self.create_timer(self.TIMER_DELAY, self.send_data)

    def send_data(self):
        input("Press any key")
        if self.idx_mode:
            ob = self.components[self.idx].lower()
            probs = np.zeros(len(self.objects))
            probs[self.objects.index(ob)] = 1
            probs += np.random.rand() * self.NOISE_LEVEL
            probs /= probs.sum()
            self.send_objects(probs)
            self.idx_mode = False
        else:
            ob = self.components[self.idx].lower()
            probs = np.zeros(len(self.objects))
            probs[self.objects.index(ob)] = 1
            probs += np.random.rand() * self.NOISE_LEVEL
            probs /= probs.sum()
            self.send_objects(probs)
            self.idx_mode = True
            self.idx += 1


    def _translate_action(self, actions: List[float]) -> Dict[str, float]:
        """Translates a list of action probabilities into a dictionary {action_name: probability}

        Args:
            actions (List[float]): list of action probabilities

        Returns:
            Dict[str, float]: Dictionary of action probabilities
        """
        return {a: v for a, v in zip(self.actions, actions)}

    def _translate_object(self, objects: List[float]) -> Dict[str, float]:
        """Translates a list of object probabilities into a dictionary {object_name: probability}

        Args:
            objects (List[float]): list of object probabilities

        Returns:
            Dict[str, float]: Dictionary of object probabilities
        """
        return {o: v for o, v in zip(self.objects, objects)}

    def send_actions(self, actions):
        aap = AssemblyActionProbability(probabilities=actions)
        self.action_pub.publish(aap)
        self.get_logger().info(f"Published: {app}")

    def send_objects(self, objects):
        aop = AssemblyObjectProbability(probabilities=objects)
        self.object_pub.publish(aop)
        self.get_logger().info(f"Published: {aop}")

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

    # def load_assembly(self, assembly_name: str):
    def load_assembly(self):
        graph, recipe_name, assembly_name, base_filename = self.builder.buildGraph(self.BUILD_FILE)
        first = {}
        second = {}
        for edge in graph.edges_iter():
            if "! >" in edge[0]:
                idx = int(edge[0][3:])
                second[idx] = edge[1]
            elif "! >" in edge[1]:
                idx = int(edge[1][3:])
                first[idx] = edge[0]
        # c = []
        # for i in range(len(first) - 1):
        #     f, s = first[i], second[i]
        #     felms = [a if f in b else b for (a, b) in [elm for elm in graph.edges(f) if not ("! >" in elm[0] or "! >" in elm[1])]]
        #     selms = [a if s in b else b for (a, b) in [elm for elm in graph.edges(s) if not ("! >" in elm[0] or "! >" in elm[1])]]
        #     print(felms, selms)
        #     if i == 0:
        #         if felms[0] == selms[0]:
        #             c += [felms[1], felms[0], selms[1]]
        #         elif felms[0] == selms[1]:
        #             c += [felms[1], felms[0], selms[0]]
        #         elif felms[1] == selms[0]:
        #             c += [felms[0], felms[1], selms[1]]
        #         elif felms[1] == selms[1]:
        #             c += [felms[0], felms[1], selms[0]]
        #     else:
        #         if felms[0] == selms[0]:
        #             c += [felms[1], selms[1]]
        #         elif felms[0] == selms[1]:
        #             c += [felms[0], selms[0]]
        #         elif felms[1] == selms[0]:
        #             c += [felms[1], selms[1]]
        #         elif felms[1] == selms[1]:
        #             c += [felms[1], selms[0]]
        # print(c)
        if self.build_assembly=='snake':
            c = ['Cube', 'Peg', 'Sphere','Peg', 'Cube','Peg', 'Cube']
            a = ['Hammer', 'Hammer', 'Hammer','Hammer', 'Hammer', 'Hammer','Hammer']
        elif self.build_assembly=='dog':
            c = ['Sphere', 'Peg', 'Cube', 'Peg', 'Cube', 'Screw', 'Wafer', 'Screw', 'Wafer']
            a = ['Hammer', 'Hammer', 'Hammer', 'Hammer', 'Hammer', 'Screwdriver', 'Nothing', 'Screwdriver', 'Nothing']
        elif self.build_assembly=='car':
            c = ['Cube', 'Peg', 'Cube', 'Peg', 'Cube', 'Screw', 'Wheel', 'Screw', 'Wheel']
            a = ['Hammer', 'Hammer', 'Hammer', 'Hammer', 'Hammer', 'Screwdriver', 'Nothing', 'Screwdriver', 'Nothing']
        return c, a


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
        tb.print_exc()


if __name__ == '__main__':
    main()
