#  -*- coding: utf-8 -*-
from rdflib.namespace import FOAF, RDFS, RDF, OWL, XMLNS, Namespace
from rdflib.extras.infixowl import Class, TermDeletionHelper
from rdflib import BNode, URIRef, Literal
from knowl import OntologyAPI, DBConfig
import yaml
from importlib.util import find_spec
import os
from uuid import uuid4
from crow_control.utils import ParamServer
try:
    import rclpy
    from rclpy.node import ParameterDescriptor
    from rcl_interfaces.msg import ParameterType
    from std_srvs.srv import Trigger
except:  # noqa
    pass  # TODO make nicer
import subprocess
import time


DB_PARAM_NAMES = ["database_host", "database_port", "database_uri", "database_name", "database_store"]
DB_PARAM_MAP = {
    "database_host": "host",
    "database_port": "port",
    "database_name": "database",
    "database_uri": "baseURL",
    "database_store": "store"
}
DB_PARAM_DESC = {
    "database_host": "Host address of the database server.",
    "database_port": "Port on which the database server is running.",
    "database_name": "Name of the database (e.g. an SQL DB name).",
    "database_uri": "Ontology base string / base namespace.",
    "database_store": "Store type, can be alchemy or fuseki."
}


class CrowtologyServer():
    """
    """
    # crowNSString = "http://imitrob.ciirc.cvut.cz/ontologies/crow#"
    # CROW = Namespace(crowNSString)
    # OWLR = Namespace("http://www.lesfleursdunormal.fr/static/_downloads/owlready_ontology.owl#")
    BACKUP_ON_CLEAR = False
    CLEAR_ON_START = True  # if clearing is False, backup has no effect (i.e. only bkp if clearing)
    ALLOW_CLIENTS_CLEARING_DATA = True  # if true, ROS service to clear/reset the DB will be created
    FUSEKI_PORT = 3030
    MODE = "ros"  # "ros" or "local" mode

    def __init__(self, config_path=None, base_onto_path=None):
        if self.MODE == "ros":  # "ros"
            self.__node = rclpy.create_node(node_name="ontology_server")
            self.log("Setting up 'ontology_server' node.")
        else:  # "local"
            self.__node = None
            self.pserver = None

        self.modulePath = find_spec("crow_ontology").submodule_search_locations[0]
        if config_path is None:
            config_path = os.path.join(self.modulePath, "..", "config", "db_config.yaml")
        if base_onto_path is None:  # load new onto
            self.base_onto_path = os.path.join(self.modulePath, "..", "data", "onto_draft_03.owl")
        else:
            self.base_onto_path = base_onto_path

        with open(config_path, 'r') as file:  # load the config
            self.__cfg = yaml.safe_load(file)

        self.__dbconf = DBConfig.factory(config_path)

        if "store" not in self.__cfg:
            self.__cfg["store"] = "alchemy"  # backwards compatibility for cfgs without store option
            self.log("No info about store type found in the config, assuming the default 'SQLAlchemy' store.")
        elif self.__cfg["store"] == "fuseki":
            self.log("Store type set to 'jena fuseki'.")
            self.fuseki_run_cmd = ""
            if "fuseki_path" in self.__cfg:
                self.log("Fuseki server path found in the config, attempting to start the server.")
                self.start_fuseki(self.__cfg["fuseki_path"])
            else:
                self.log("Fuseki server path NOT found in the config, assuming server was started manually.")
        elif self.__cfg["store"] == "alchemy":
            self.log("Store type set to 'SQLAlchemy'.")
        else:
            raise Exception(f'Unknown store type {self.__cfg["store"]}!')

        # time.sleep(10)
        self.__onto = OntologyAPI(self.__dbconf)
        if self.CLEAR_ON_START:
            self.clear_data()

        if self.__dbconf.store == "fuseki":
            self.compactify_fuseki()
        self.log("Ontology DB is running.")

    def compactify_fuseki(self):
        # fuseki_tool_path = '~/packages/apache-jena-4.2.0/'
        fuseki_tool_path = self.fuseki_path.replace("fuseki-", "") # try to extract fuseki tools from the fuseki_path
        fuseki_tool_path = os.path.expanduser(fuseki_tool_path)
        compactor_path = os.path.join(fuseki_tool_path, 'bin/tdb2.tdbcompact')
        if not os.path.exists(compactor_path):
            self.log(f"Could not find Fuseki compactor at '{compactor_path}'")
            return
        try:
            ret = subprocess.run(' '.join([compactor_path, '--loc=' + os.path.join(self.fuseki_path + 'run/')]), stdout=subprocess.PIPE, shell=True, check=True)
            if ret.returncode > 0:
                raise Exception(f"Tried to compactify the Fuseki database returned an error code: {ret.returncode}.\nThe output of the run command: {ret.stdout}")
            else:
                self.log(f"{ret.stdout.decode('utf-8')}\nFuseki database compactified.")
            #imitrob@aurora:~/packages/apache-jena-4.2.0/bin$ ./tdb2.tdbcompact --loc=../../apache-jena-fuseki-4.2.0/run/
        except BaseException as e:
            self.log(f"Tried to compactify the Fuseki database, but failed because: {e}")

    def clear_data(self):
        try:
            if len(self.onto) > 0:  # try to make a backup
                if self.BACKUP_ON_CLEAR:
                    try:
                        bkp_path = os.path.join(self.modulePath, "..", "data", "backup", f"bkp_{uuid4()}.owl")
                        self.onto.graph.serialize(bkp_path, format="xml")
                    except Exception as e:
                        self.log(f"Tried backing up the ontology but failed because: {e}")
                try:
                    if self.__dbconf.store == "alchemy":
                        self.onto.destroy("I know what I am doing")
                        self.onto.closelink()
                        self.__onto = OntologyAPI(self.__dbconf)
                    elif self.__dbconf.store == "fuseki":
                        self.onto.update("""
                        DELETE {
                            ?s ?p ?o
                        }
                        WHERE {
                            ?s ?p ?o .
                        }
                        """)
                            # (None, None, None))  # remove all triples
                        # self.onto.remove((None, None, None))  # remove all triples
                    else:
                        raise Exception(f"Unknown DB store type: {self.__dbconf.store}")
                except Exception as e:
                    self.log(f"Tried cleaning up the ontology but failed because: {e}")
        except Exception as e:
            self.log(f"Tried checking the size of ontology, but failed because: {e}")
        self.onto.mergeFileIntoDB(self.base_onto_path)

    def log(self, msg):
        if self.node is not None:
            self.node.get_logger().info(msg)
        else:
            print(msg)

    def start_fuseki(self, fuseki_path):
        self.fuseki_path = os.path.expanduser(fuseki_path)
        self.fuseki_run_cmd = os.path.join(self.fuseki_path, 'fuseki')
        fuseki_env = {
                "FUSEKI_ARGS": f"--port {self.FUSEKI_PORT} --update --tdb2 --loc run /{self.__dbconf.database}"
                # "FUSEKI_ARGS": f"--port {self.FUSEKI_PORT} --update --tdb2"
            }
        # print(fuseki_env)
        if os.path.exists(self.fuseki_run_cmd):
            self.log(f'Running fuseki as service from {self.fuseki_run_cmd}...')
        else:
            raise Exception(f'Fuseki executable not found in: {self.fuseki_run_cmd}!')
        if self.get_fuseki_status():
            self.log('Fuseki is already running! Trying to kill it first!')
            ret = subprocess.run(' '.join([self.fuseki_run_cmd, 'stop']), stdout=subprocess.PIPE, shell=True, check=True, env=fuseki_env)
            if ret.returncode > 0:
                raise Exception(f"Fuseki returned an error code: {ret.returncode}.\nThe output of the run command: {ret.stdout.decode('utf-8')}")
            else:
                if ret.returncode > 0:
                    raise Exception(f"Trying to stop Fuseki returned an error code: {ret.returncode}.\nThe output of the run command: {ret.stdout}")
                else:
                    self.log(f"{ret.stdout.decode('utf-8')}\nFuseki service stopped.")
        else:
            ret = subprocess.run(' '.join([self.fuseki_run_cmd, 'start']), stdout=subprocess.PIPE, shell=True, check=True, env=fuseki_env)
        if ret.returncode > 0:
            raise Exception(f"Fuseki returned an error code: {ret.returncode}.\nThe output of the run command: {ret.stdout.decode('utf-8')}")
        else:
            if "fail" in str(ret.stdout):
                self.log(f"Fuseki service somehow failed but the returncode was 0 (indicating no error). Read the output of the run command and decide what to do:\n{ret.stdout.decode('utf-8')}")
            else:
                self.log(f"{ret.stdout.decode('utf')}\nFuseki service started on port {self.FUSEKI_PORT}.")

    def get_fuseki_status(self) -> bool:
        ret = subprocess.run(' '.join([self.fuseki_run_cmd, 'status']), stdout=subprocess.PIPE, shell=True, check=True)
        if ret.returncode > 0:
            raise Exception(f"Trying to get Fuseki status returned an error code: {ret.returncode}.\nThe output of the run command: {ret.stdout.decode('utf-')}")
        else:
            return "is running" in str(ret.stdout)

    def start_onto_server(self):
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
        if self.ALLOW_CLIENTS_CLEARING_DATA:
            self.clearing_service = self.node.create_service(Trigger, "reset_database", self.clear_data_service_cb)
        self.log("Created server with params:")
        for k, v, d in parameters:
            self.log(f"\t{k}: {v}")

    def clear_data_service_cb(self, request, response):
        try:
            self.clear_data()
        except BaseException as e:
            response.success = False
            response.message = str(e)
        else:
            response.success = True
        return response

    def start_param_server(self):
        self.pserver = ParamServer()

    @property
    def onto(self):
        return self.__onto

    @property
    def node(self):
        return self.__node

    def destroy(self):
        if self.pserver is not None:
            self.pserver.destroy()
        if self.node is not None:
            self.node.destroy_node()
        if self.__cfg["store"] == "fuseki" and self.fuseki_run_cmd and self.get_fuseki_status():
            ret = subprocess.run(' '.join([self.fuseki_run_cmd, 'stop']), stdout=subprocess.PIPE, shell=True, check=True)
            if ret.returncode > 0:
                raise Exception(f"Trying to stop Fuseki returned an error code: {ret.returncode}.\nThe output of the run command: {ret.stdout}")
            else:
                self.log(f"{ret.stdout.decode('utf-8')}\nFuseki service stopped.")


def main_ros(args=[]):
    rclpy.init()

    cs = CrowtologyServer()
    cs.start_param_server()
    cs.start_onto_server()
    try:
        rclpy.spin(cs.node)
    except KeyboardInterrupt as ke:
        print("User requested shutdown.")
    finally:
        cs.destroy()

def main():  # run from console as standalone python script
    # TODO Process args
    cs = CrowtologyServer()
    input("Hit enter to exit (makes sense, right?)...")  # so that the script does not exit
    cs.destroy()


if __name__ == "__main__":
    main()
