from rdflib.namespace import FOAF, RDFS, RDF, OWL, XMLNS, Namespace
from rdflib.extras.infixowl import Class
from rdflib import BNode, URIRef, Literal
from knowl import OntologyAPI, DBConfig
import os

import rclpy
from rclpy.node import Node
from threading import Thread
from rcl_interfaces.srv import GetParameters
from uuid import uuid4
import yaml

ONTO_SERVER_NAME = "ontology_server"


class CrowtologyClient():
    DB_PARAM_NAMES = ["database_host", "database_port", "database_uri", "database_name"]

    def __init__(self, credential_file_path=None, local_mode=False):
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
        self.__client_id = uuid4()  # id in case this client needs to be identified in ROS

        if credential_file_path is None:
            modulePath = find_spec("crow_ontology").submodule_search_locations[0]
            credential_file_path = os.path.join(modulePath, "..", "config", "db_config.yaml")

        self.__local_mode = local_mode
        if self.local_mode:  # LOCAL MODE
            self.__onto = OntologyAPI(credential_file_path)
        else:  # ROS MODE
            rclpy.init()
            self.__node = rclpy.create_node(f"onto_client_{self.client_id}")

            with open(credential_file_path, 'r') as file:
                cfg = yaml.safe_load(file)

            # try to get the database parameters (host, port, ...)
            self.__db_params = self.get_db_params()
            self.__config = DBConfig(
                host=self.__db_params["database_host"],
                port=self.__db_params["database_port"],
                baseURL=self.__db_params["database_uri"],
                database=self.__db_params["database_name"],
                # namespaces=
            )
            self.__config.setCredentials(username=cfg["username"], password=cfg["password"])
            self.__onto = OntologyAPI(self.__config)
            self.node_thread = Thread(target=lambda : rclpy.spin(self.__node), name="node_runner")
            self.node_thread.daemon = True
            self.__node.context.on_shutdown(self._on_shutdown)
            self.node_thread.start()

        # bind some basic namespaces?
        # self.__onto.bind("crow", self.CROW)  # this is not good, overwrites the base namespace

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

        request = GetParameters.Request(names=self.DB_PARAM_NAMES)
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.__node, future, 10)
        if not future.done():
            raise Exception("Could not retrieve the database parameters from the ROS server node.")
        return {k: p.string_value for k, p in zip(self.DB_PARAM_NAMES, future.result().values)}

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

    def _on_shutdown(self):
        self.onto.closelink()
