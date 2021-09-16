import json
import os
import pickle
import time
from typing import Dict, List
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from crow_msgs.msg import AssemblyObjectProbability, AssemblyActionProbability
from rclpy.executors import MultiThreadedExecutor
from crow_control.utils.yaml_to_graph import AssemblyGraphMaker
from crow_ontology.crowracle_client import CrowtologyClient
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from crow_msgs.msg import StampedString

from crow_nlp.nlp_crow.processing.NLProcessor import NLProcessor
from crow_nlp.nlp_crow.processing.ProgramRunner import ProgramRunner
from crow_nlp.nlp_crow.modules.UserInputManager import UserInputManager

from crow_control.utils import ParamClient
from importlib.util import find_spec


class AssemblyPlanner(Node):
    ASSEMBLY_ACTION_TOPIC = 'assembly_action'
    ASSEMBLY_OBJECT_TOPIC = 'assembly_object'

    def __init__(self):
        super().__init__("assembly_planner")
        # connect to onto
        self.crowracle = CrowtologyClient(node=self)
        self.onto = self.crowracle.onto
        self.max_node_prev = -1
        self.LANG='cs'
        # TODO save after building the tree the tree and just load the saved object
        build_file = 'data/build_snake'
        # build_file = os.path.join(find_spec("crow_control").submodule_search_locations[0], "data", "build_snake")
        onto_file = "../../ontology/onto_draft_02.owl"
        self.ui = UserInputManager(language = self.LANG)
        self.templ_det = self.ui.load_file('templates_detection.json')
        self.am = AssemblyGraphMaker(build_file)
        self.pclient = ParamClient()
        self.pclient.define("processor_busy_flag", False) # State of the sentence processor
        self.pclient.define("halt_nlp", False) # If true, NLP input should be halted
        self.pclient.define("silent_mode", 1) # Set by the user (level of the talking - 1 Silence, 2 - Standard speech, 3 - Debug mode/Full speech)
        self.pclient.define("ready_for_next_sentence", True) # If true, sentence processor can process and send next command
        self.pclient.define("can_start_talking", True) # If true, can start playing buffered sentences
        self.pclient.define("det_obj", "-")
        self.pclient.define("det_command", "-")
        self.pclient.define("det_obj_name", "-")
        self.pclient.define("det_obj_in_ws", "-")
        self.pclient.define("status", "-")

        if os.path.exists(self.am.graph_name):
            self.gp = pickle.load(open(self.am.graph_name, 'rb'))
            print('loading graph from a file')
        else:
            print('building a new graph for the given assembly')
            g, g_name, assembly_name, base_filename = self.am.build_graph(onto_file)
            self.gp = self.am.prune_graph(g)        # get object / action dicts
        self.objects = [o[2:].lower() for o in dir(AssemblyObjectProbability) if o.startswith("O_")]
        self.actions = [a[2:].lower() for a in dir(AssemblyActionProbability) if a.startswith("A_")]

        # callbacks
        self.create_subscription(AssemblyActionProbability, self.ASSEMBLY_ACTION_TOPIC, self.action_cb, 10)
        self.create_subscription(AssemblyObjectProbability, self.ASSEMBLY_OBJECT_TOPIC, self.object_cb, 10)
        qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
        self.assembly_publisher = self.create_publisher(StampedString, "/nlp/command_planner", qos)

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
        Po = dict(self._translate_object(objects.probabilities))
        self.am.update_graph(self.gp, Po)
        max_node = self.am.detect_most_probable_state(self.gp)
        [next_node, obj_to_add] = self.am.detect_next_state(self.gp, max_node)

        if max_node == self.max_node_prev:
            print('same node as previously. No action.')
        else:
            # self.parameters = ['action', 'action_type', 'target', 'target_type']
            # self.target = []  # object to pick
            # self.target_type = 'onto_uri'
            # self.action_type = self.templ_det[self.lang]['pick']
            # self.action = CommandType.PICK
            # ###TODO: replaced by default robot behaviour (pick and home?)
            # self.location = [0.53381, 0.18881, 0.22759]  # temporary "robot default" position
            # self.location_type = 'xyz'
            obj_to_add_lang = self.templ_det[self.LANG][str.lower(obj_to_add)]
            input_sentence = 'polož '+ obj_to_add_lang + " na stůl"
            print(input_sentence)
            input_sentence = input_sentence.lower()
            nl_processor = NLProcessor(language=self.LANG, client=self.crowracle)
            program_template = nl_processor.process_text(input_sentence)
            print()
            print("Program Template")
            print("--------")
            print(program_template)
            robot_program = self.run_program(program_template)
            data = []
            try:
                template = program_template.root.children[0].template
                # if (template is None) or (not template.is_filled()):
                #     self.wait_then_talk()
                #     continue
                dict1 = template.get_inputs()
                template_type = dict1.get('action_type', '-')
                target_obj = dict1.get('target')
                if target_obj is not None:
                    if hasattr(target_obj, "flags") and 'last_mentioned' in target_obj.flags:
                        # TODO add last mentioned correferenced object and then delete this and adjust in ObjectGrounder
                        target_obj = None
                        dict1['target'] = None
                if target_obj is not None:
                    if len(target_obj) > 1:
                        self.wait_then_talk()
                    # @TODO: ask for specification, which target_obj to choose
                    object_detected = target_obj
                    found_in_ws = True
                else:
                    object_detected = '-'
                    found_in_ws = False

                self.send_status("zpracovavam", template_type, object_detected, found_in_ws)
                if dict1.get('action', False):
                    data.append(dict1)
                    actions = json.dumps(data)
                    msg = StampedString()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.data = actions
                    print(f'will publish {msg.data}')
            # {"action": 64, "action_type": "uka\u017e", "target_type": "onto_uri",
            #  "target": ["http://imitrob.ciirc.cvut.cz/ontologies/crow#cube_holes_1",
            #             "http://imitrob.ciirc.cvut.cz/ontologies/crow#cube_holes_2",
            #             "http://imitrob.ciirc.cvut.cz/ontologies/crow#cube_holes_3"],
            #  "target_ph_cls": "http://imitrob.ciirc.cvut.cz/ontologies/crow#Cube"}
                    self.assembly_publisher.publish(msg)
                    self.send_status("pozadavek odeslan", template_type, object_detected, found_in_ws)
                    self.wait_then_talk()
                    self.pclient.processor_busy_flag = False
                else:
                    self.send_status("neznamy")
                    self.wait_then_talk()
            except AttributeError as  e:
                print('No template found error')
                self.send_status("neznamy prikaz")
                self.wait_then_talk()
        self.max_node_prev = max_node

    def run_program(self, program_template):
        program_runner = ProgramRunner(language = self.LANG, client = self.crowracle)
        robot_program = program_runner.evaluate(program_template)
        print()
        print("Grounded Program")
        print("--------")
        print(robot_program)
        return robot_program

    def send_status(self, status="zadejte pozadavek", template_type="-", object_detected="-", found_in_ws=False):
        self.pclient.det_obj = object_detected
        self.pclient.det_command = template_type
        #self.pclient.det_obj_name = ??? # use crowtology client function, see visualizator
        self.pclient.det_obj_in_ws = found_in_ws # change to string 'ano', 'ne'?, see visualizator
        self.pclient.status = status

    def wait_then_talk(self):
        if self.pclient.silent_mode > 1:
            while self.pclient.can_start_talking == False:
                time.sleep(0.2)
            self.pclient.can_start_talking = False
        self.ui.buffered_say(flush=True, level=self.pclient.silent_mode)
        self.pclient.can_start_talking = True

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
