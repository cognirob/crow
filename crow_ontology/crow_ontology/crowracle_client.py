from rdflib.namespace import FOAF, RDFS, RDF, OWL, XMLNS, Namespace
from rdflib.extras.infixowl import Class
from rdflib import BNode, URIRef, Literal
from rdflib.plugins.sparql import prepareQuery
from knowl import OntologyAPI, DBConfig
import os
from importlib.util import find_spec
from uuid import uuid4
import yaml
from crow_ontology.crowracle_server import DB_PARAM_NAMES, DB_PARAM_MAP
try:
    import rclpy
    from threading import Thread
    from rcl_interfaces.srv import GetParameters
except:  # noqa
    pass


ONTO_SERVER_NAME = "ontology_server"


class CrowtologyClient():

    CROW = Namespace("http://imitrob.ciirc.cvut.cz/ontologies/crow#")
    _tangible_leaf_query = prepareQuery("""SELECT ?cls
        WHERE {
            ?cls rdfs:subClassOf+ crow:TangibleObject .
            FILTER NOT EXISTS {?nan rdfs:subClassOf ?cls . }
        }""",
                                        initNs={"owl": OWL, "crow": CROW}
                                        )
    _tangible_query = prepareQuery("""SELECT ?cls
        WHERE {
            ?cls rdfs:subClassOf+ crow:TangibleObject .
        }""",
                                   initNs={"owl": OWL, "crow": CROW}
                                   )
    _present_query = prepareQuery("""SELECT ?obj
        WHERE {
            ?obj crow:hasId ?c .
            ?obj rdf:type ?cls .
            ?cls rdfs:subClassOf* crow:TangibleObject .
        }""",
                                   initNs={"owl": OWL, "crow": CROW}
                                   )

    def __init__(self, *, credential_file_path=None, node=None, local_mode=False):
        """Creates and ontology client object. The client can be started in ROS mode,
        where it retrieves connection data (DB address, etc.) from a running server node
        (this node is not running the ontology, just maintains it and stores the connection
        data). Or it can be started in local mode, where all the connection data are retrieved
        from a local config file. In both cases, the constructor needs access to a config file
        with at least the credentials to connect to the database.

        *The credentials are not provided by the server node!*

        Args:
            credential_file_path (str, optional): Path to the file with credentials
            or full configuration (in case of local mode). If None, the code will try to look
            for the configuration file in some default location. Defaults to None.
            local_mode (bool, optional): Whether to run in local mode. Defaults to False.
        """
        self.__client_id = str(uuid4()).replace("-", "_")  # id in case this client needs to be identified in ROS
        self.__uses_external_node = False

        if credential_file_path is None:
            modulePath = find_spec("crow_ontology").submodule_search_locations[0]
            credential_file_path = os.path.join(modulePath, "..", "config", "db_config.yaml")

        self.__local_mode = local_mode
        if self.local_mode:  # LOCAL MODE
            self.__onto = OntologyAPI(credential_file_path)
        else:  # ROS MODE
            if node is None:
                if not rclpy.ok():
                    rclpy.init()
                self.__node = rclpy.create_node(f"onto_client_{self.client_id}")
            else:
                self.__uses_external_node = True
                self.__node = node

            with open(credential_file_path, 'r') as file:
                cfg = yaml.safe_load(file)

            # try to get the database parameters (host, port, ...)
            self.__db_params = self.get_db_params()
            initial_config = {DB_PARAM_MAP[k]: v for k, v in self.__db_params.items()}
            self.__node.get_logger().info(str(initial_config))
            self.__config = DBConfig(
                **initial_config
                # namespaces=
            )
            self.__config.setCredentials(username=cfg["username"], password=cfg["password"])
            self.__onto = OntologyAPI(self.__config)

            self.__node.context.on_shutdown(self._on_shutdown)
            if not self.__uses_external_node:
                self.__node_thread = Thread(target=lambda : rclpy.spin(self.__node), name="node_runner")
                self.__node_thread.daemon = True
                self.__node_thread.start()

        # bind some basic namespaces?
        self.__onto.bind("crow", self.CROW)  # this is not good, overwrites the base namespace
        self.__onto.bind("owl", OWL)

    def get_db_params(self):
        """Tries to get the connection parameters from the server node, if run in ROS mode

        Raises:
            Exception: Timeout trying to contact the server node (the node is probably not running)
            Exception: Timeout trying to retrieve the parameters.

        Returns:
            dict: The connection parameters
        """
        client = self.__node.create_client(GetParameters, f'/{ONTO_SERVER_NAME}/get_parameters')
        if not client.wait_for_service(10):
            raise Exception("Could not locate the onto server ROS node! Did you start it yet?")

        request = GetParameters.Request(names=DB_PARAM_NAMES)
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.__node, future, timeout_sec=10)
        if not future.done():
            raise Exception("Could not retrieve the database parameters from the ROS server node.")
        return {k: p.string_value for k, p in zip(DB_PARAM_NAMES, future.result().values)}

    def getTangibleObjectClasses(self, mustBeLeaf=True):
        """Return classes of all TangibleObjects (i.e. physical objects that can be present on the workspace)

        Args:
            mustBeLeaf (bool, optional): If True, only leaf classes are returned. That is,
            no general classes (e.g. "Workpice", "Tool") will be returned. Defaults to True.

        Returns:
            list: List of RDFLib terms describing the classes. Use str(result[i]) to turn into string.
        """
        qres = self.onto.query(self._tangible_leaf_query if mustBeLeaf else self._tangible_query)
        return list(qres)

    def getTangibleObjects(self):
        """Lists physical objects present on the workspace

        Returns:
            list: The objects.
        """
        # res = self.onto.triples((None, self.CROW.hasColor, None))
        # objects = []
        # for tangible, _, id in res:
        #     for _, _, tcls in self.onto.triples((tangible, RDF.type, None)):
        #         if self.CROW.TangibleObject in self.onto.transitive_objects(tcls, RDFS.subClassOf):
        #             objects.append(tangible)
        #             break
        # return objects
        res = self.onto.query(self._present_query)
        return list(res)

    @property
    def client_id(self):
        return self.__client_id

    @property
    def onto(self):
        return self.__onto

    @property
    def local_mode(self):
        return self.__local_mode

    def close(self):
        """Use this to properly disconnect from the ontology.
        """
        self.onto.closelink()
        if not self.local_mode:
            rclpy.shutdown()
            self.__node.destroy_node()

    def _on_shutdown(self):
        self.onto.closelink()
