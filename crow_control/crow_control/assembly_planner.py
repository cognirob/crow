import os
import pickle
from typing import Dict, List
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from crow_msgs.msg import AssemblyObjectProbability, AssemblyActionProbability
from rclpy.executors import MultiThreadedExecutor
from crow_control.utils.yaml_to_graph import AssemblyGraphMaker
from crow_ontology.crowracle_client import CrowtologyClient

class AssemblyPlanner(Node):
    ASSEMBLY_ACTION_TOPIC = 'assembly_action'
    ASSEMBLY_OBJECT_TOPIC = 'assembly_object'

    def __init__(self):
        super().__init__("assembly_planner")
        # connect to onto
        self.crowracle = CrowtologyClient(node=self)
        self.onto = self.crowracle.onto
        # TODO save after building the tree the tree and just load the saved object
        build_file = 'data/build_snake'
        onto_file = "../../ontology/onto_draft_02.owl"
        self.am = AssemblyGraphMaker(build_file)
        if os.path.exists(self.am.graph_name):
            gp = pickle.load(open(self.am.graph_name, 'rb'))
            print('loading graph from a file')
        else:
            print('building a new graph for the given assembly')
            g, g_name, assembly_name, base_filename = self.am.build_graph(onto_file)
            gp = self.am.prune_graph(g)        # get object / action dicts
        self.objects = [o[2:].lower() for o in dir(AssemblyObjectProbability) if o.startswith("O_")]
        self.actions = [a[2:].lower() for a in dir(AssemblyActionProbability) if a.startswith("A_")]

        # callbacks
        self.create_subscription(AssemblyActionProbability, self.ASSEMBLY_ACTION_TOPIC, self.action_cb, 10)
        self.create_subscription(AssemblyObjectProbability, self.ASSEMBLY_OBJECT_TOPIC, self.object_cb, 10)

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

    def action_cb(self, actions):
        self.get_logger().info(f"Got some action probabilities: {str(self._translate_action(actions.probabilities))}")

    def object_cb(self, objects):
        self.get_logger().info(f"Got some object probabilities: {str(self._translate_object(objects.probabilities))}")

def main():
    rclpy.init()
    ap = AssemblyPlanner()
    try:
        n_threads = 2
        mte = MultiThreadedExecutor(num_threads=n_threads, context=rclpy.get_default_context())
        rclpy.spin_once(ap, executor=mte)
        ap.get_logger().info("ready")
        rclpy.spin(ap, executor=mte)
        ap.destroy_node()
    except KeyboardInterrupt:
        ap.destroy_node()
        print("User requested shutdown.")
    except BaseException as e:
        print(f"Some error had occured: {e}")


if __name__ == '__main__':
    main()
