from rdflib.namespace import FOAF, RDFS, RDF, OWL, XMLNS, Namespace
from rdflib.extras.infixowl import Class
from rdflib import BNode, URIRef, Literal
from knowl import OntologyAPI
import yaml
from importlib.util import find_spec
import os
from uuid import uuid4
try:
    import rclpy
    from rclpy.node import ParameterDescriptor
    from rcl_interfaces.msg import ParameterType
except:  # noqa
    pass  # TODO make nicer

DB_PARAM_NAMES = ["database_host", "database_port", "database_uri", "database_name"]
DB_PARAM_MAP = {
    "database_host": "host",
    "database_port": "port",
    "database_name": "database",
    "database_uri": "baseURL"
}
DB_PARAM_DESC = {
    "database_host": "Host address of the database server.",
    "database_port": "Port on which the database server is running.",
    "database_name": "Name of the database (e.g. an SQL DB name).",
    "database_uri": "Ontology base string / base namespace."
}


class CrowtologyServer():
    """
    """
    # crowNSString = "http://imitrob.ciirc.cvut.cz/ontologies/crow#"
    # CROW = Namespace(crowNSString)
    # OWLR = Namespace("http://www.lesfleursdunormal.fr/static/_downloads/owlready_ontology.owl#")

    def __init__(self, config_path=None, base_onto_path=None):
        if config_path is None:
            modulePath = find_spec("crow_ontology").submodule_search_locations[0]
            config_path = os.path.join(modulePath, "..", "config", "db_config.yaml")

        with open(config_path, 'r') as file:  # load the config
            self.__cfg = yaml.safe_load(file)

        self.__onto = OntologyAPI(config_path)

        if len(self.onto) > 0:  # try to make a backup
            bkp_path = os.path.join(modulePath, "..", "data", "backup", f"bkp_{uuid4()}.owl")
            self.onto.graph.serialize(bkp_path, format="xml")
            self.onto.destroy("I know what I am doing")
            self.onto.closelink()
            self.__onto = OntologyAPI(config_path)

        if base_onto_path is None:  # load new onto
            base_onto_path = os.path.join(modulePath, "..", "data", "onto_draft_03.owl")
        self.onto.mergeFileIntoDB(base_onto_path)

    def start_param_server(self):
        self.__node = rclpy.create_node(node_name="ontology_server")
        self.node.get_logger().info("Setting up 'ontology_server' node.")
        invmap = {v: k for k, v in DB_PARAM_MAP.items()}
        parameters = [(invmap[k], v,
                       ParameterDescriptor(
                           type=ParameterType.PARAMETER_STRING, description=DB_PARAM_DESC[invmap[k]]))
                      for k, v in self.__cfg.items() if k in invmap.keys()
                      ]
        self.__node.declare_parameters(
            namespace=None,
            parameters=parameters
        )
        self.node.get_logger().info("Created server with params:")
        for k, v, d in parameters:
            self.node.get_logger().info(f"\t{k}: {v}")

    @property
    def onto(self):
        return self.__onto

    @property
    def node(self):
        return self.__node


def main_ros(args=[]):
    rclpy.init()

    cs = CrowtologyServer()
    cs.start_param_server()

    rclpy.spin(cs.node)
    cs.node.destroy_node()

def main():  # run from console as standalone python script
    # TODO Process args
    cs = CrowtologyServer()
    input("Hit enter to exit (makes sense, right?)...")  # so that the script does not exit


if __name__ == "__main__":
    main()
