from typing import List
from numpy.core.numeric import NaN
from rdflib.namespace import FOAF, RDFS, RDF, OWL, XMLNS, XSD, Namespace
from rdflib.extras.infixowl import Class
from rdflib import BNode, URIRef, Literal
from rdflib.plugins.sparql import prepareQuery
from knowl import OntologyAPI, DBConfig
import os
from importlib.util import find_spec
from uuid import uuid4
import numpy as np
import yaml
from crow_ontology.crowracle_server import DB_PARAM_NAMES, DB_PARAM_MAP
from crow_vision_ros2.utils.test_point_in_polyhedron import test_in_hull
try:
    import rclpy
    from rclpy.node import Node
    from threading import Thread
    from rcl_interfaces.srv import GetParameters
except:  # noqa
    pass
from threading import RLock
from unicodedata import normalize
import time
from std_srvs.srv import Trigger


ONTO_SERVER_NAME = "ontology_server"
ONTO_IRI = "http://imitrob.ciirc.cvut.cz/ontologies/crow"
OWL_READY = "http://www.lesfleursdunormal.fr/static/_downloads/owlready_ontology.owl"

class CrowtologyClient():

    CROW = Namespace(f"{ONTO_IRI}#")
    OWL_READY_NS = Namespace(f"{OWL_READY}#")

    _query_add_object = """PREFIX rdf: <http://www.w3.org/1999/02/22-rdf-syntax-ns#>
        prefix owl: <http://www.w3.org/2002/07/owl#>
        prefix rdfs: <http://www.w3.org/2000/01/rdf-schema#>
        prefix crow: <http://imitrob.ciirc.cvut.cz/ontologies/crow#>

        INSERT {
            ?individual ?prop ?value .
            ?individual crow:hasAbsoluteLocation ?loc_name .
            ?individual crow:hasPclDimensions ?pcl_name .
            ?loc_name a crow:xyzAbsoluteLocation .
            ?loc_name crow:x ?loc_x .
            ?loc_name crow:y ?loc_y .
            ?loc_name crow:z ?loc_z .
            ?pcl_name a crow:xyzPclDimensions .
            ?pcl_name crow:x ?pcl_x .
            ?pcl_name crow:y ?pcl_y .
            ?pcl_name crow:z ?pcl_z .
            ?individual crow:hasId ?adder_id .
            ?individual crow:hasUuid ?uuid .
            ?individual crow:hasTimestamp ?stamp .
        }

        WHERE {
            ?template ?prop ?value .
            FILTER NOT EXISTS { ?template crow:hasUuid ?uuid }
            FILTER NOT EXISTS { ?any crow:hasAbsoluteLocation ?value }
            FILTER NOT EXISTS { ?any crow:hasPclDimensions ?value }
        }"""
    _query_add_object_no_template = """PREFIX rdf: <http://www.w3.org/1999/02/22-rdf-syntax-ns#>
        prefix owl: <http://www.w3.org/2002/07/owl#>
        prefix rdfs: <http://www.w3.org/2000/01/rdf-schema#>
        prefix crow: <http://imitrob.ciirc.cvut.cz/ontologies/crow#>

        INSERT {
            ?individual ?prop ?value .
            ?individual crow:hasAbsoluteLocation ?loc_name .
            ?individual crow:hasPclDimensions ?pcl_name .
            ?loc_name a crow:xyzAbsoluteLocation .
            ?loc_name crow:x ?loc_x .
            ?loc_name crow:y ?loc_y .
            ?loc_name crow:z ?loc_z .
            ?pcl_name a crow:xyzPclDimensions .
            ?pcl_name crow:x ?pcl_x .
            ?pcl_name crow:y ?pcl_y .
            ?pcl_name crow:z ?pcl_z .
            ?individual crow:hasId ?adder_id .
            ?individual crow:hasUuid ?uuid .
            ?individual crow:hasTimestamp ?stamp .
        }

        WHERE {
            ?template ?prop ?value ;
                    crow:hasDetectorName ?det_name .
            FILTER NOT EXISTS { ?template crow:hasUuid ?uuid }
            FILTER NOT EXISTS { ?any crow:hasAbsoluteLocation ?value }
            FILTER NOT EXISTS { ?any crow:hasPclDimensions ?value }
        }"""
    _query_tangible_leaf = """SELECT ?cls
        WHERE {
            ?cls rdfs:subClassOf+ crow:TangibleObject .
            FILTER NOT EXISTS {?nan rdfs:subClassOf ?cls .}
            FILTER NOT EXISTS {?cls crow:hasId ?id .}
        }"""
    _query_tangible = """SELECT ?cls
        WHERE {
            ?cls rdfs:subClassOf+ crow:TangibleObject .
        }"""
    _query_present = """SELECT DISTINCT ?obj
        WHERE {
            ?obj crow:hasId ?c .
            ?obj rdf:type ?cls .
            ?cls rdfs:subClassOf* crow:TangibleObject .
        }"""
    _query_present_and_disabled_nocls = """SELECT DISTINCT ?obj
        WHERE {
            ?obj crow:hasTimestamp ?c .
        }"""
    _query_present_nocls = """SELECT DISTINCT ?obj
        WHERE {
            ?obj crow:hasId ?c .
        }"""
    _query_check_time_enable_disable = """SELECT DISTINCT ?obj ?stamp ?enabled
        WHERE {
            ?obj crow:hasTimestamp ?stamp .
            BIND(EXISTS{?obj crow:hasId ?id} AS ?enabled)
        }"""
    _query_present_props = """SELECT ?obj ?id ?cls ?col ?colczname ?colenname ?czname ?enname ?x ?y ?z
        WHERE {
            ?obj crow:hasId ?id .
            ?obj rdf:type ?cls .
            ?cls rdfs:subClassOf* crow:TangibleObject .
            ?obj crow:hasColor ?col .
            ?col crow:hasNlpNameEN ?colenname .
            ?col crow:hasNlpNameCZ ?colczname .
            ?obj crow:hasNlpNameEN ?enname .
            ?obj crow:hasNlpNameCZ ?czname .
            ?obj crow:hasAbsoluteLocation ?loc .
            ?loc crow:x ?x .
            ?loc crow:y ?y .
            ?loc crow:z ?z .
        }"""
    _query_get_location = """SELECT ?x ?y ?z
        WHERE {
            ?obj crow:hasAbsoluteLocation ?loc .
            ?loc crow:x ?x .
            ?loc crow:y ?y .
            ?loc crow:z ?z .
        }"""
    _query_present_tangible_location = """SELECT DISTINCT ?obj ?x ?y ?z
        WHERE {
    		?obj crow:hasId ?id .
            ?obj crow:hasAbsoluteLocation ?loc .
            ?loc crow:x ?x .
            ?loc crow:y ?y .
            ?loc crow:z ?z .
        }"""
    _query_get_dimensions = """SELECT ?x ?y ?z
        WHERE {
            ?obj crow:hasPclDimensions ?pcl .
            ?pcl crow:x ?x .
            ?pcl crow:y ?y .
            ?pcl crow:z ?z .
        }"""
    _query_get_timestamp = """SELECT ?stamp
        WHERE {
            ?obj crow:hasTimestamp ?stamp .
        }"""
    _query_actions_props = """SELECT ?obj ?name ?start ?stop ?uuid
        WHERE {
            ?obj rdf:type crow:Action .
            ?obj crow:hasName ?name .
            ?obj crow:hasStartTimestamp ?start .
            ?obj crow:hasStopTimestamp ?stop .
            ?obj crow:hasUuid ?uuid .
        }"""
    _query_positions_props = """SELECT ?obj ?name
        WHERE {
            ?obj rdf:type crow:Position .
            ?obj crow:hasName ?name .
        }"""
    _query_storages_props = """SELECT ?obj ?name
        WHERE {
            ?obj rdf:type crow:StorageSpace .
            ?obj crow:hasName ?name .
        }"""
    _query_marker_group_propsEN = """SELECT ?obj ?name ?dict_num ?size ?seed ?id ?square_len
        WHERE {
            ?obj rdf:type crow:MarkerGroup .
            ?obj crow:hasNlpNameEN ?name .
            ?obj crow:hasMarkerDictAmount ?dict_num .
            ?obj crow:hasMarkerSize ?size .
            ?obj crow:hasSeed ?seed .
            ?obj crow:hasMarkerId ?id .
            ?obj crow:hasSquareLength ?square_len .
        }"""
    _query_marker_group_propsCZ = """SELECT ?obj ?name ?dict_num ?size ?seed ?id ?square_len
        WHERE {
            ?obj rdf:type crow:MarkerGroup .
            ?obj crow:hasNlpNameCZ ?name .
            ?obj crow:hasMarkerDictAmount ?dict_num .
            ?obj crow:hasMarkerSize ?size .
            ?obj crow:hasSeed ?seed .
            ?obj crow:hasMarkerId ?id .
            ?obj crow:hasSquareLength ?square_len .
        }"""
    _query_colors = """SELECT ?obj
        WHERE {
            ?obj rdf:type crow:NamedColor .
        }"""
    _query_colors_nlp = """SELECT ?name
        WHERE {
            ?obj a crow:NamedColor .
            ?obj ?language ?name .
        }"""
    _query_filter_properties = """SELECT DISTINCT ?name ?col ?sigma
        WHERE {
            ?obj crow:hasDetectorName ?name .
            ?obj crow:hasFilterColor ?col .
            ?obj crow:hasSigma ?sigma .
        }"""
    _query_area_polyhedron = """
        SELECT ?x ?y ?z

        WHERE {
            ?area crow:hasPolyhedron ?poly .
            ?poly crow:hasPoint3D ?pt .
            ?pt crow:x ?x .
            ?pt crow:y ?y .
            ?pt crow:z ?z .
        }"""
    _query_area_polygon = """
        SELECT ?x ?y ?z

        WHERE {
            ?area crow:hasPolygon ?poly .
            ?poly crow:hasPoint3D ?pt .
            ?pt crow:x ?x .
            ?pt crow:y ?y .
            ?pt crow:z ?z .
        }"""
    _query_main_areas = """
        SELECT DISTINCT ?area ?x ?y ?z

        WHERE {
            ?area a crow:StorageSpace .
            ?area crow:hasPolyhedron ?poly .
            ?poly crow:hasPoint3D ?pt .
            ?pt crow:x ?x .
            ?pt crow:y ?y .
            ?pt crow:z ?z .
            FILTER EXISTS { ?area crow:areaType 'main' }
        }"""
    _query_all_tangible_nlp = """
        SELECT ?name

        WHERE {
            ?cls rdfs:subClassOf+ crow:TangibleObject .
            FILTER NOT EXISTS { ?nan rdfs:subClassOf ?cls . }
            FILTER NOT EXISTS { ?cls crow:hasId ?id . }
            ?cls ?language ?name .
        }"""
    _query_present_tangible_nlp = """
        SELECT ?name

        WHERE {
            ?obj crow:hasId ?c .
            ?obj rdf:type ?cls .
            ?cls rdfs:subClassOf* crow:TangibleObject .
            ?cls ?language ?name .
        }"""
    _query_disable_object = """PREFIX rdf: <http://www.w3.org/1999/02/22-rdf-syntax-ns#>
        prefix owl: <http://www.w3.org/2002/07/owl#>
        prefix rdfs: <http://www.w3.org/2000/01/rdf-schema#>
        prefix crow: <http://imitrob.ciirc.cvut.cz/ontologies/crow#>

        DELETE { ?individual crow:hasId ?id }
        INSERT { ?individual crow:disabledId ?id }
        WHERE {
            ?individual crow:hasId ?id .
        }"""
    _query_enable_object = """PREFIX rdf: <http://www.w3.org/1999/02/22-rdf-syntax-ns#>
        prefix owl: <http://www.w3.org/2002/07/owl#>
        prefix rdfs: <http://www.w3.org/2000/01/rdf-schema#>
        prefix crow: <http://imitrob.ciirc.cvut.cz/ontologies/crow#>

        DELETE { ?individual crow:disabledId ?id }
        INSERT { ?individual crow:hasId ?id }
        WHERE {
            ?individual crow:disabledId ?id .
        }"""
    _query_delete_object = """PREFIX rdf: <http://www.w3.org/1999/02/22-rdf-syntax-ns#>
        prefix owl: <http://www.w3.org/2002/07/owl#>
        prefix rdfs: <http://www.w3.org/2000/01/rdf-schema#>
        prefix crow: <http://imitrob.ciirc.cvut.cz/ontologies/crow#>

        DELETE {
            ?s ?p ?o .
        }
        WHERE {
            ?s ?p ?o .
            FILTER (?s = ?individual || ?o = ?individual)
        }"""
    _query_update_object = """PREFIX rdf: <http://www.w3.org/1999/02/22-rdf-syntax-ns#>
        prefix owl: <http://www.w3.org/2002/07/owl#>
        prefix rdfs: <http://www.w3.org/2000/01/rdf-schema#>
        prefix crow: <http://imitrob.ciirc.cvut.cz/ontologies/crow#>

        DELETE {
            ?individual crow:hasTimestamp ?old_stamp .
            ?loc crow:x ?old_x .
            ?loc crow:y ?old_y .
            ?loc crow:z ?old_z .
            ?pcl crow:x ?old_pcl_x .
            ?pcl crow:y ?old_pcl_y .
            ?pcl crow:z ?old_pcl_z .
        }
        INSERT {
            ?individual crow:hasTimestamp ?new_stamp .
            ?loc crow:x ?new_x .
            ?loc crow:y ?new_y .
            ?loc crow:z ?new_z .
            ?pcl crow:x ?new_pcl_x .
            ?pcl crow:y ?new_pcl_y .
            ?pcl crow:z ?new_pcl_z .
        }
        WHERE {
            ?individual crow:hasTimestamp ?old_stamp .
            ?individual crow:hasAbsoluteLocation ?loc .
            ?individual crow:hasPclDimensions ?pcl .
            ?loc crow:x ?old_x .
            ?loc crow:y ?old_y .
            ?loc crow:z ?old_z .
            ?pcl crow:x ?old_pcl_x .
            ?pcl crow:y ?old_pcl_y .
            ?pcl crow:z ?old_pcl_z .
        }"""

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
        self.lock = RLock()
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
            self.__db_params = self.__get_db_params()
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

        if self.__config.store == "alchemy":  # SQLAlchemy store needs "prepareQuery"
            setattr(self, "prepareQuery", prepareQuery)
            # prepare queries
            queries = [(getattr(self, qs), qs) for qs in dir(self) if qs.startswith("_query_")]
            for q, qs in queries:
                if "INSERT" in q or "DELETE" in q:
                    continue
                # print(q)
                setattr(self, qs, self.prepareQuery(q, initNs={"owl": OWL, "crow": self.CROW, "rdf": RDF, "rdfs": RDFS}))
        elif self.__config.store == "fuseki":  # fuseki uses plain string
            setattr(self, "prepareQuery", lambda qs, *args, **kwargs: qs)
        else:
            raise Exception(f"Unknown store type {self.__config.store}!")

        # bind some basic namespaces?
        self.__onto.bind("crow", self.CROW)  # this is not good, overwrites the base namespace
        self.__onto.bind("owl", OWL)

        self.db_reset_client = self.node.create_client(Trigger, "reset_database")
        if not self.db_reset_client.wait_for_service(5):
            self.db_reset_client = None

    def prepareQuery(self, query, *args, **kwargs):
        raise Exception("This should be overriden in the init function!")

    @property
    def node(self) -> Node:
        return self.__node

    def __get_db_params(self):
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

    def reset_database(self):
        if self.db_reset_client is not None:
            response = self.db_reset_client.call(Trigger.Request())
            if response.success:
                print("Database reset successful.")
            else:
                print(f"Could not reset the database: {response.message}")
        else:
            print("Cannot reset the database, server does not provide service for it.")


    def get_filter_object_properties(self):
        """Return dict of numbered dicts with info about objects relevant to filter

        Returns:
            res_list (dict of dicts): Object properties (name, sigma, color) for filter
        """
        qres = list(self.onto.query(self._query_filter_properties))
        res_list = {}
        qres.sort(key = lambda i: i["name"])
        idx = 0
        for idx, g in enumerate(qres):
            res_dict = {}
            res_dict["name"] = g["name"].toPython()
            res_dict["sigma"] = g['sigma'].toPython()
            res_dict["color"] = np.fromstring(g["col"].toPython().strip('[]').strip("''"), dtype=float, sep=' ')
            res_list[idx] = res_dict
        names = [res_list[res]['name'] for res in res_list.keys()]
        for name in ['kuka', 'kuka_gripper', 'hand']:
            if name not in names:
                idx += 1
                res_dict = {}
                res_dict['name'] = name
                res_dict['sigma'] = res_list[idx-1]['sigma']
                res_dict['color'] = np.array([0.004, 0.004, 0.004])
                res_list[idx] = res_dict
        return res_list

    def getTangibleObjectClasses(self, mustBeLeaf=True):
        """Return classes of all TangibleObjects (i.e. physical objects that can be present on the workspace)

        Args:
            mustBeLeaf (bool, optional): If True, only leaf classes are returned. That is,
            no general classes (e.g. "Workpice", "Tool") will be returned. Defaults to True.

        Returns:
            list: List of RDFLib terms describing the classes. Use str(result[i]) to turn into string.
        """
        qres = self.onto.query(self._query_tangible_leaf if mustBeLeaf else self._query_tangible)
        return [g[0] for g in qres]

    def getTangibleObjects(self):
        """Lists physical objects present on the workspace

        Returns:
            list: The objects.
        """
        res = self.onto.query(self._query_present)
        return [g["obj"] for g in res]

    def getTangibleObjects_timestamp(self):
        """Lists physical objects present on the workspace

        Returns:
            list: The objects.
        """
        res = self.onto.query(self._query_check_time_enable_disable)
        # return res
        # print(list(res))
        return [(g["obj"], g["stamp"], g["enabled"].toPython()) for g in res]

    def getTangibleObjectsProps(self):
        """Lists physical objects present on the workspace together with their properties

        Returns:
            res_list (list of dicts): The objects and their properties
        """
        res = self.onto.query(self._query_present_props)
        res_list = []
        for g in res:
            res_dict = {}
            res_dict["uri"] = g["obj"]
            res_dict["id"] = g["id"].toPython()
            res_dict["color"] = g["col"]
            res_dict["color_nlp_name_CZ"] = g["colczname"].toPython()
            res_dict["color_nlp_name_EN"] = g["colenname"].toPython()
            res_dict["nlp_name_CZ"] = g["czname"].toPython()
            res_dict["nlp_name_EN"] = g["enname"].toPython()
            try:
                res_dict["absolute_location"] = [float(q) for q in [g["x"], g["y"], g["z"]]]
            except:
                res_dict["absolute_location"] = [str(q) for q in [g["x"], g["y"], g["z"]]]
            res_list.append(res_dict)

        return res_list

    def getTangibleObjects_nocls(self):
        """Lists physical objects present on the workspace NO CLS

        Returns:
            list: The objects.
        """
        res = self.onto.query(self._query_present_nocls)
        return [g["obj"] for g in res]

    def getTangibleObjects_disabled_nocls(self):
        """Lists physical objects present or disabled on the workspace NO CLS

        Returns:
            list: The objects.
        """
        res = self.onto.query(self._query_present_and_disabled_nocls)
        return [g["obj"] for g in res]

    def getColors(self):
        """Lists all colors in the database

        Returns:
            list: The colors (URIRefs).
        """
        res = self.onto.query(self._query_colors)
        return [g["obj"] for g in res]

    def get_obj_of_properties(self, obj_cls, uri_dict, all=False):
        """Get URI object of properties specified by URIs

        Args:
            obj_cls (URIRef): class of object
            uri_dict (dict): keys=Python names of properties, values=URIs of objects or values for Literals
            all (bool): search among all or only in the scene objects

        Returns:
            list of URIRefs: objects, 0...N
        """
        q_string = 'SELECT DISTINCT ?sub WHERE { ?sub rdf:type ?cls . '
        initBindings = {}
        if obj_cls is not None:
            initBindings['cls'] = obj_cls
        if all == False:
            q_string += '?sub crow:hasId ?id . '

        for i, (k, v) in enumerate(uri_dict.items()):
            k_property = self.get_prop_from_name(k)
            if k_property is None:
                continue
            initBindings['prop{}'.format(i)] = k_property
            if v is not None:
                initBindings['obj{}'.format(i)] = v
            q_string += '?sub ?prop{} ?obj{} . '.format(i, i)
        q_string += '}'

        q = self.prepareQuery(q_string, initNs={"rdf": RDF, "crow":self.CROW})
        subjects = self.onto.query(q, initBindings=initBindings)
        return [x['sub'] for x in subjects]

    def get_obj_of_id(self, id):
        """Get URI object of specified id

        Args:
            id (str): id of object

        Returns:
            list of URIRefs: objects, 0...N
        """
        #prop_range = list(self.onto.objects(subject=CROW.hasId, predicate=RDFS.range))[0]
        objects = list(self.onto.subjects(self.CROW.hasId, Literal(id)))
        return objects

    def get_obj_of_uuid(self, uuid):
        """Get URI object of specified id

        Args:
            uuid (str): uuid of object

        Returns:
            list of URIRefs: objects, 0...N
        """
        objects = list(self.onto.subjects(self.CROW.hasUuid, Literal(uuid)))
        return objects

    def get_id_of_obj(self, uri):
        """Get id of URI object

        Args:
            uri URIRefs: objects

        Returns:
            id (str): id of object
        """
        #prop_range = list(self.onto.objects(subject=CROW.hasId, predicate=RDFS.range))[0]
        ids = list(self.onto.objects(uri, self.CROW.hasId))
        if len(ids) > 0: # assume obj has exactly one id
            return ids[0].toPython()
        else:
            return None

    def get_uuid_of_obj(self, uri):
        """Get uuid of URI object

        Args:
            uri URIRefs: objects

        Returns:
            uuid (str): uuid of object
        """
        ids = list(self.onto.objects(uri, self.CROW.hasUuid))
        if len(ids) > 0: # assume obj has exactly one uuid
            return ids[0].toPython()
        else:
            return None

    # 7
    def get_location_of_obj(self, uri):
        """Get absolute location of URI object

        Args:
            uri (URIRef): URI of obj, 1

        Returns:
            list of floats: xyz location, 1x3
        """
        result = self.onto.query(self._query_get_location, initBindings={
            "obj": uri
        })
        if len(result) > 0: # assume obj has max one location
            try: # expect floats
                loc = [float(c) for c in list(result)[0]]
            except: # but may be None (if not localized yet)
                loc = [str(c) for c in list(result)[0]]
            return loc
        else:
            return [None]*3

    def get_pcl_dimensions_of_obj(self, uri):
        """Get dimensions of pcl of detected object

        Args:
            uri (URIRef): URI of obj, 1

        Returns:
            list of floats: xyz dimension, 1x3
        """
        result = self.onto.query(self._query_get_dimensions, initBindings={
            "obj": uri
        })
        if len(result) > 0: # assume obj has max one location
            try: # expect floats
                loc = [float(c) for c in list(result)[0]]
            except: # but may be None (if not localized yet)
                loc = [str(c) for c in list(result)[0]]
            return loc
        else:
            return [None]*3

    def get_timestamp_of_obj(self, uri):
        """Get timestamp of detected object

        Args:
            uri (URIRef): URI of obj, 1

        Returns:
            timestamp in XSD.Timestamp format
        """
        result = self.onto.query(self._query_get_timestamp, initBindings={
            "obj": uri
        })
        if len(result) > 0: # assume obj has max one location
            stamp = str(list(result)[0][0])
            return stamp
        else:
            return None

    def get_fixed_dimensions_of_obj(self, uri):
        """Get dimensions of detected object, specified by 3D models

        Args:
            uri (URIRef): URI of obj, 1

        Returns:
            list of floats: xyz dimension, 1x3
        """
        dim_obj = list(self.onto.objects(uri, self.CROW.hasBoxDimensions))
        if len(dim_obj) > 0: # assume obj has max one dimensions
            try: # expect floats
                dim = [float(list(self.onto.objects(dim_obj[0], x))[0]) for x in [self.CROW.x, self.CROW.y, self.CROW.z]]
            except: # but may be None (if not localized yet)
                dim = [str(list(self.onto.objects(dim_obj[0], x))[0]) for x in [self.CROW.x, self.CROW.y, self.CROW.z]]
            return dim
        else:
            return [None]*3

    # 6
    def get_obj_of_color(self, uri):
        """Get URI object of specified URI color

        Args:
            uri (URIRef): URI of color, 1

        Returns:
            list of URIRefs: objects, 0...N
        """
        objects = list(self.onto.subjects(self.CROW.hasColor, uri))
        return objects

    # 5
    def get_color_of_obj(self, uri):
        """Get URI color of URI object

        Args:
            uri (URIRef): URI of obj, 1

        Returns:
            list of URIRefs: URI color of URI object, 1
        """
        color = list(self.onto.objects(uri, self.CROW.hasColor))
        if len(color) > 0:
            return color # assume obj has only one color
        else: # this obj does not have color
            return None

    # 4
    def get_uri_from_nlp(self, name):
        """Get URI of something specified by nlp name

        Args:
            name (str): name of obj (may be class, or individual, ...), 1

        Returns:
            list of URIRefs: URI of given the thing of given name, 0...N
        """
        result_entities = []
        entities = []
        entities.append(list(self.onto.subjects(self.CROW.hasNlpNameEN, Literal(name, datatype=XSD.string))))
        entities.append(list(self.onto.subjects(self.CROW.hasNlpNameCZ, Literal(name, datatype=XSD.string))))
        entities = sum(entities,[])
        # multiple colors may have the same nlp name
        # classes of objects (not objects) have nlp name -> find all objects of these classes
        for ent in entities:
            obj_of_ent_class = list(self.onto.subjects(RDF.type, ent))
            if len(obj_of_ent_class) > 0: # ent is a class
                for obj in obj_of_ent_class:
                    result_entities.append(obj) # append URIobject of this class
            elif ent not in result_entities:
                result_entities.append(ent) # ent is a color, append the URIcolor
        return(result_entities)

    # 3
    def get_nlp_from_uri(self, uri, language='EN'):
        """Get nlp name of something specified by URIRef

        Args:
            uri (URIRef): URI of obj (may be class, or individual, ...), 1
            language (str): nlp names in which language

        Returns:
            list of strings: nlp names of the given uri, 0...N
        """
        nlp_name_property = self.CROW.hasNlpNameEN if language == 'EN' else self.CROW.hasNlpNameCZ
        if type(uri)== list:
            uri = uri[0]
        query = f"""
        PREFIX crow:    <http://imitrob.ciirc.cvut.cz/ontologies/crow#>
        PREFIX rdfs: <http://www.w3.org/2000/01/rdf-schema#>
        PREFIX rdf: <http://www.w3.org/1999/02/22-rdf-syntax-ns#>
        PREFIX owl: <http://www.w3.org/2002/07/owl#>

        SELECT DISTINCT ?name
        
        WHERE {{
            BIND({uri.n3()} AS ?obj)
            ?obj a ?cls .
            BIND(IF(?cls = crow:NamedColor || ?cls = owl:Class, ?obj, ?cls) AS ?target)
            ?target {nlp_name_property.n3()} ?name .
        }}"""
        nlp_name_uri = list(self.onto.query(query))
        if len(nlp_name_uri) < 1: # no nlp name -> create new from the uri string
            nlp_name = [uri.split('#')[-1]]
            self.onto.add((uri, nlp_name_property, Literal(nlp_name[0], datatype=XSD.string)))
        else:
            nlp_name = [x[0].toPython() for x in nlp_name_uri]
        return nlp_name

    def get_prop_from_name(self, py_name):
        """Get property URIRef specifiend by python name

        Args:
            py_name (str): python name of property

        Returns:
            property (URIRef): property of given python name
        """
        prop = list(self.onto.subjects(self.OWL_READY_NS.python_name, Literal(py_name)))
        if len(prop) > 0:
            if len(prop) > 1:
                #@TODO: logger info
                print("More than one property of name {} found, continue with the first one.".format(py_name))
            return prop[0]
        else:
            return None

    def get_name_from_prop(self, prop):
        """Get python name of property specified by URIRef

        Args:
            prop (URIRef): URI of property

        Returns:
            name (str): python name of the given property
        """
        name = list(self.onto.objects(prop, self.OWL_READY_NS.python_name))
        if len(name) > 0:
            if len(name) > 1:
                #@TODO: logger info
                print("More than one name for property {} found, continue with the first one.".format(prop))
            return name[0]
        else:
            return None

    # A "which all objects are in the ontology?"
    def get_all_tangible_nlp(self, language='EN') -> List[str]:
        """Get nlp names of all tangible objects

        Args:
            language (str): nlp names in which language

        Returns:
            list of strings: nlp names of all tangible objects, 0...N
        """
        # all_tangible = self.getTangibleObjectClasses()
        # all_tangible_nlp = []
        # for tangible in all_tangible:
        #     all_tangible_nlp.append(self.get_nlp_from_uri(tangible, language=language))
        result = self.onto.query(self._query_all_tangible_nlp, initBindings={"language": self.CROW.hasNlpNameEN if language == 'EN' else self.CROW.hasNlpNameCZ})
        return [x.name.toPython() for x in result]

    # B "which objects are in the scene?"
    def get_tangible_nlp(self, language='EN'):
        """Get nlp names of tangible objects in the scene

        Args:
            language (str): nlp names in which language

        Returns:
            list of strings: nlp names of tangible objects in the scene, 0...N
        """
        # all_tangible = self.getTangibleObjects()
        # all_tangible_nlp = []
        # for tangible in all_tangible:
        #     all_tangible_nlp.append(self.get_nlp_from_uri(tangible, language=language))
        # return all_tangible_nlp
        result = self.onto.query(self._query_present_tangible_nlp, initBindings={"language": self.CROW.hasNlpNameEN if language == 'EN' else self.CROW.hasNlpNameCZ})
        return [x.name.toPython() for x in result]

    def get_colors_nlp(self, language='EN'):
        """Get nlp names of all colors in the database

        Args:
            language (str): nlp names in which language

        Returns:
            list of strings: nlp names of colors in the database, 0...N
        """
        # all_colors = self.getColors()
        # all_colors_nlp = []
        # for color in all_colors:
        #     all_colors_nlp.append(self.get_nlp_from_uri(color, language=language))
        # return all_colors_nlp

        result = self.onto.query(self._query_colors_nlp, initBindings={"language": self.CROW.hasNlpNameEN if language == 'EN' else self.CROW.hasNlpNameCZ}, initNs={"rdf": RDF})
        return [x.name.toPython() for x in result]

    def get_all_tools(self, all=False):
        """
        Returns a list of all tools in the workspace.

        Args:
            all (bool): search among all or only in the scene objects
        """
        if all == False:
            q_string = 'SELECT ?sub WHERE { ?sub rdf:type ?cls . ?cls rdfs:subClassOf* crow:Tool . ?sub crow:hasId ?id . }'
        elif all == True:
            q_string = 'SELECT ?sub WHERE { ?sub rdfs:subClassOf+ crow:Tool . FILTER NOT EXISTS {?nan rdfs:subClassOf ?sub . }}'
        q = self.prepareQuery(q_string, initNs={"rdf": RDF, "rdfs": RDFS, "crow":self.CROW})
        subjects = self.onto.query(q)
        return [x['sub'] for x in subjects]

    # C "what color does the cube have?"
    def get_color_of_obj_nlp(self, name, language='EN'):
        """Get nlp name of color of an object specified by nlp name

        Args:
            name (str): nlp name of object, 1
            language (str): nlp names in which language

        Returns:
            list of strings: nlp names of colors the object has, 0...N
        """
        uris = self.get_uri_from_nlp(name) # multiple objects may have same nlp name
        result_entities = []
        for uri in uris: # find colors of each object
            colors = self.get_color_of_obj(uri) # assume one obj may have more than one color
            for color in colors:
                color_names = self.get_nlp_from_uri(color, language=language) # color may have multiple nlp names
                result_entities.append(color_names)
        result_entities = list(set(sum(result_entities, []))) # concat all possible colors
        return result_entities

    # D "which objects (in the scene) are red?"
    def get_obj_of_color_nlp(self, name, language='EN', all=False):
        """Get nlp name of objects of given color specified by nlp name

        Args:
            name (str): nlp name of color, 1
            language (str): nlp names in which language
            all (bool): search among all or only in the scene objects

        Returns:
            list of strings: nlp names of all objects of specified color, 0...N
        """
        uris = self.get_uri_from_nlp(name) # uris of colors, multiple may have same nlp name
        obj_names = []
        if all == False:
            objs_in_scene = self.getTangibleObjects() # go through objects in the scene only
        for uri in uris:
            obj_uris = self.get_obj_of_color(uri) # multiple objects may have the same color
            for obj_uri in obj_uris:
                if all == True:
                    obj_names.append(self.get_nlp_from_uri(obj_uri, language=language))
                elif obj_uri in objs_in_scene:
                    obj_names.append(self.get_nlp_from_uri(obj_uri, language=language))
        obj_names = list(set(sum(obj_names, []))) # concat objects names
        return obj_names

    # E "where (in the scene) is cube?"
    def find_obj_nlp(self, name, all=False):
        """Get absolute location of object specified by nlp name

        Args:
            name (str): nlp name of object, 1
            all (bool): search among all or only in the scene objects

        Returns:
            list of lists of floats: xyz locations of all objects of given nlp name, 0x3...Nx3
        """
        uris = self.get_uri_from_nlp(name) # multiple obj may have the same nlp name
        obj_locations = []
        if all == False:
            objs_in_scene = self.getTangibleObjects() # go through objects in the scene only
        for uri in uris:
            if all == True:
                obj_locations.append(self.get_location_of_obj(uri))
            elif uri in objs_in_scene:
                obj_locations.append(self.get_location_of_obj(uri))
        return obj_locations

    # F "where (in the scene) are green objects?"
    def find_obj_of_color_nlp(self, name, all=False):
        """Get absolute location of object specified by nlp name of its color

        Args:
            name (str): nlp name of color, 1
            all (bool): search among all or only in the scene objects

        Returns:
            list of lists of floats: xyz locations of all objects of given color specified by nlp name, 0x3...Nx3
        """
        uris = self.get_uri_from_nlp(name) # uris of colors, multiple may have same nlp name
        obj_locations = []
        if all == False:
            objs_in_scene = self.getTangibleObjects() # go through objects in the scene only
        for uri in uris:
            obj_uris = self.get_obj_of_color(uri) # multiple objects may have the same color
            for obj_uri in obj_uris:
                if all == True:
                    obj_locations.append(self.get_location_of_obj(obj_uri))
                elif obj_uri in objs_in_scene:
                    obj_locations.append(self.get_location_of_obj(obj_uri))
        return obj_locations

    def set_last_mentioned_object(self, object):
        """Sets an object as last mentioned.

        Args:
            object (URIRef): the object which should be set as "last mentioned"
        """
        subjects = self.get_last_mentioned_object()
        if subjects is not None:
            for sub in subjects:
                self.onto.set((sub, self.CROW.isLastMentioned, False))
        self.onto.set((object, self.CROW.isLastMentioned, True))

    def get_last_mentioned_object(self):
        """
        Returns the object which was set as last mentioned or None if there is no such object.
        """
        subjects = list(self.onto.subjects(self.CROW.isLastMentioned, True))
        if len(subjects) > 0:
            return subjects
        else:
            return None

    def get_uri_from_str(self, str):
        """
        Returns correct URIRef from the URI string.

        Args:
            str (str): URI string
        Returns:
            obj_uri (URIRef): URI
        """
        new_ns = Namespace(f"{str.split('#')[0]}#")
        obj_uri = new_ns[str.split('#')[-1]]
        return obj_uri

    def get_world_name_from_uri(self, uri):
        """Get annotation property "world_name" from URI

        Args:
            str (URIRef): URI of the object

        Returns:
            str
        """
        try:
            _query_wname = f"""PREFIX rdf: <http://www.w3.org/1999/02/22-rdf-syntax-ns#>
                prefix owl: <http://www.w3.org/2002/07/owl#>
                prefix rdfs: <http://www.w3.org/2000/01/rdf-schema#>
                prefix crow: <http://imitrob.ciirc.cvut.cz/ontologies/crow#>

                SELECT DISTINCT ?wname
                WHERE {{
                    {uri.n3()} rdf:type ?cls .
                    ?cls crow:world_name ?wname .
                }}"""
            result = self.onto.query(_query_wname)
            world_name = str(list(result)[0][0])
        except:
            return "UNKNOWN"
        return world_name

    def get_target_from_type(self, obj_type):
        obj_type = obj_type.n3()
        query = f"""PREFIX rdf: <http://www.w3.org/1999/02/22-rdf-syntax-ns#>
            prefix owl: <http://www.w3.org/2002/07/owl#>
            prefix rdfs: <http://www.w3.org/2000/01/rdf-schema#>
            prefix crow: <http://imitrob.ciirc.cvut.cz/ontologies/crow#>

            SELECT DISTINCT ?x ?y ?z ?sx ?sy ?sz ?wname
            WHERE {{
                ?obj rdf:type ?cls .
                FILTER(?cls = {obj_type})
                ?cls crow:world_name ?wname .
                ?obj crow:hasAbsoluteLocation ?loc .
                ?loc crow:x ?x .
                ?loc crow:y ?y .
                ?loc crow:z ?z .
                FILTER(?x != "None" && ?y != "None" && ?z != "None")
                ?obj crow:hasPclDimensions ?pcl .
                ?pcl crow:x ?sx .
                ?pcl crow:y ?sy .
                ?pcl crow:z ?sz .
            }}
            LIMIT 1"""
        result = self.onto.query(query)
        if len(result) == 0:
            return
        r = list(result)[0]
        *xyz, sx, sy, sz = [x.toPython() for x in r[:-1]]
        return xyz, [sx, sy, sz], str(r[-1])

    def get_target_from_uri(self, uri):
        obj = uri.n3()
        query = f"""PREFIX rdf: <http://www.w3.org/1999/02/22-rdf-syntax-ns#>
            prefix owl: <http://www.w3.org/2002/07/owl#>
            prefix rdfs: <http://www.w3.org/2000/01/rdf-schema#>
            prefix crow: <http://imitrob.ciirc.cvut.cz/ontologies/crow#>

            SELECT DISTINCT ?x ?y ?z ?sx ?sy ?sz ?wname
            WHERE {{
                {obj} rdf:type ?cls .
                ?cls crow:world_name ?wname .
                {obj} crow:hasAbsoluteLocation ?loc .
                ?loc crow:x ?x .
                ?loc crow:y ?y .
                ?loc crow:z ?z .
                {obj} crow:hasPclDimensions ?pcl .
                ?pcl crow:x ?sx .
                ?pcl crow:y ?sy .
                ?pcl crow:z ?sz .
            }}"""
        result = self.onto.query(query)
        if len(result) == 0:
            return
        r = list(result)[0]
        *xyz, sx, sy, sz = [x.toPython() for x in r[:-1]]
        return xyz, [sx, sy, sz], str(r[-1])

    def getCurrentAction(self):
        """Get current action's name and last update time

        Returns:
            res_dict (dictionary): The current action's name and update time
        """
        res_dict = {}
        try:
            res_dict['name'] = next(self.onto.objects(self.CROW.CurrentAction, self.CROW.hasName)).toPython()
            res_dict['timestamp'] = str(next(self.onto.objects(self.CROW.CurrentAction, self.CROW.hasStopTimestamp)))
        except:
            self.__node.get_logger().info("There is no current action in the loaded database.")
        return res_dict

    def getActionsProps(self):
        """Lists actions detected in the session together with their properties

        Returns:
            res_list (list of dicts): The actions and their properties
        """
        res = self.onto.query(self._query_actions_props)
        res_list = []
        for g in res:
            res_dict = {}
            res_dict["uri"] = g["obj"]
            res_dict["name"] = g["name"].toPython()
            res_dict["uuid"] = g["uuid"].toPython()
            res_dict["start_timestamp"] = g["start"].toPython()
            res_dict["stop_timestamp"] = g["stop"].toPython()
            res_list.append(res_dict)
        return res_list

    def getPositionsProps(self):
        """Lists positions detected in the session together with their properties

        Returns:
            res_list (list of dicts): The positions and their properties
        """
        res = self.onto.query(self._query_positions_props)
        res_list = []
        for g in res:
            res_dict = {}
            res_dict["uri"] = g["obj"]
            res_dict["name"] = g["name"].toPython()
            res_list.append(res_dict)
        return res_list

    def getStoragesProps(self):
        """Lists storages detected in the session together with their properties

        Returns:
            res_list (list of dicts): The storages and their properties
        """
        res = self.onto.query(self._query_storages_props)
        res_list = []
        for g in res:
            res_dict = {}
            res_dict["uri"] = g["obj"]
            res_dict["name"] = g["name"].toPython()
            res_list.append(res_dict)
        return res_list

    def getMarkerGroupProps(self, name, language='EN'):
        """Lists properties of marker group

        Returns:
            res_dict: The properties
        """
        if language == 'CZ':
            quick_dirty_conv = {"modr": "modrá", "červen": "červená", "zelen": "zelená"}
            for inp, out in quick_dirty_conv.items():
                if inp in name:
                    name = out
                    break
            res = self.onto.query(self._query_marker_group_propsCZ, initBindings={'name': name})
        elif language == 'EN':
            res = self.onto.query(self._query_marker_group_propsEN, initBindings={'name': name})
        else:
            "Invalid language choice (EN or CZ), taking default EN option"
            res = self.onto.query(self._query_marker_group_propsEN, initBindings={'name': name})
        res_dict = {}
        res_dict["id"] = []
        for g in res:
            res_dict["id"].append(g["id"].toPython())
            res_dict["uri"] = g["obj"]
            res_dict["name"] = str(g["name"])
            res_dict["dict_num"] = g["dict_num"].toPython()
            res_dict["size"] = g["size"].toPython()
            res_dict["square_len"] = g["square_len"].toPython()
            res_dict["seed"] = g["seed"].toPython()
        return res_dict

    def get_polyhedron(self, uri):
        """Get location of points in polyhedron defining a storage space

        Args:
            uri (URIRef): URI of the storage space

        Returns:
            polyhedron (list of lists of floats): xyz locations
        """
        res = self.onto.query(self._query_area_polyhedron, initBindings={'area': uri})
        if len(res) > 0:
            area_pts = [[float(x), float(y), float(z)] for x, y, z in res]
            return area_pts
        else:
            raise Exception(f"Error trying to get polyhedron of storage space {uri}! The result was:\n{list(res)}")
            # area_pts = [[None]]

    def get_polygon(self, uri):
        """Get location of points in polygon defining a storage space

        Args:
            uri (URIRef): URI of the storage space

        Returns:
            polygon (list of lists of floats): xyz locations
        """
        print(uri)
        res = self.onto.query(self._query_area_polygon, initBindings={'area': uri})
        if len(res) > 0:
            area_pts = [[float(x), float(y), float(z)] for x, y, z in res]
            return area_pts
        else:
            raise Exception(f"Error trying to get polygon of storage space {uri}! The result was:\n{list(res)}")

    def get_area_centroid(self, uri):
        """Get location of centroid of a storage space

        Args:
            uri (URIRef): URI of the storage space

        Returns:
            centroid (lists of floats): xyz location of centroid
        """
        centroid = self.get_location_of_obj(uri)
        return centroid

    def test_obj_in_area(self, obj_uri, area_uri):
        """Test if object is located inside the given area (defined as storage space)

        Args:
            obj_uri (URIRef): URI of the object
            area_uri (URIRef): URI of the storage space

        Returns:
            res (bool): True when object lies inside area, False otherwise
        """
        area_poly = self.get_polyhedron(area_uri)
        obj_location = self.get_location_of_obj(obj_uri)
        # print(f"* obj_location: {obj_location}")
        # print(f"* area_poly: {area_poly}")
        if area_poly != [[None]]:
            res = test_in_hull(obj_location, area_poly)
            return res
        else:
            return None

    def pair_objects_to_areas_wq(self, verbose=False):
        """ Test if objects are located inside any area, if so - append that object
        to that area

        Args:
            verbose (bool): If verbose is set to True, write out all triples who have
                predicate self.CROW.insideOf

        Returns:
            -
        """
        areas = self.onto.query(self._query_main_areas)
        # print(list(areas))
        objs = self.onto.query(self._query_present_tangible_location)
        # print(list(objs))

        # build areas
        dict_areas = {}
        for ap in areas:
            a, *points = ap
            a = a.n3()
            if a not in dict_areas:
                dict_areas[a] = []
            dict_areas[a].append([p.toPython() for p in points])

        inserts = ""
        deletes = ""
        wheres = ""
        for obj in objs:
            o, *opoints = obj
            o = o.n3()
            s = f"{o} crow:insideOf ?a .\n"
            deletes += s
            wheres += s
            opoints = [p.toPython() for p in opoints]
            for aname, apoints in dict_areas.items():
                if test_in_hull(opoints, apoints):
                    inserts += f"{o} crow:insideOf {aname} .\n"
                    inserts += f"{aname} crow:contains {o} .\n"
        query = f"""PREFIX rdf: <http://www.w3.org/1999/02/22-rdf-syntax-ns#>
            prefix owl: <http://www.w3.org/2002/07/owl#>
            prefix rdfs: <http://www.w3.org/2000/01/rdf-schema#>
            prefix crow: <http://imitrob.ciirc.cvut.cz/ontologies/crow#>
            DELETE {{
                {deletes}
            }}\n"""
        query += f"""WHERE {{
            {wheres}
        }}"""
        self.onto.update(query)
        if len(inserts) > 0:
            query = f"""PREFIX rdf: <http://www.w3.org/1999/02/22-rdf-syntax-ns#>
                prefix owl: <http://www.w3.org/2002/07/owl#>
                prefix rdfs: <http://www.w3.org/2000/01/rdf-schema#>
                prefix crow: <http://imitrob.ciirc.cvut.cz/ontologies/crow#>

                INSERT DATA {{
                    {inserts}
                }}\n"""
            self.onto.update(query)

    def get_objects_from_area(self, area_name):
        if type(area_name) is URIRef:
            area_name = area_name.n3()
        else:
            area_name = self.CROW[area_name].n3()
        query = f"""SELECT DISTINCT ?obj
        WHERE {{
            ?obj crow:insideOf {area_name} .
        }}"""
        # print(query)
        result = self.onto.query(query)
        return [u[0] for u in list(result)]

    
    def get_objects_with_poses_from_area(self, area_name):
        if type(area_name) is URIRef:
            area_name = area_name.n3()
        else:
            area_name = self.CROW[area_name].n3()
        
        query = f"""SELECT DISTINCT ?obj ?x ?y ?z ?pcl_x ?pcl_y ?pcl_z
        WHERE {{
            ?obj crow:insideOf {area_name} .
            ?obj crow:hasAbsoluteLocation ?loc .
            ?loc crow:x ?x .
            ?loc crow:y ?y .
            ?loc crow:z ?z .
            ?obj crow:hasPclDimensions ?pcl_name . 
            ?pcl_name crow:x ?pcl_x .
            ?pcl_name crow:y ?pcl_y .
            ?pcl_name crow:z ?pcl_z .
        }}"""
        # print(query)
        result = self.onto.query(query)
        print(list(result))
        if len(list(result)) > 0:
            u = list(result)
            obj = u[0][0]
            x = float(u[0][1])
            y = float(u[0][2])
            z = float(u[0][3])
            dx = float(u[0][4])
            dy = float(u[0][5])
            dz = float(u[0][6])
            # obj_type = u[0][7]


            return obj, x, y, z, dx, dy, dz, 1
        return None, NaN, NaN, NaN, NaN, NaN, NaN, 0

        # data sample for query
        # data = [(rdflib.term.URIRef('http://imitrob.ciirc.cvut.cz/ontologies/crow#hammer_od_713'), rdflib.term.Literal('0.76065123', datatype=rdflib.term.URIRef('http://www.w3.org/2001/XMLSchema#float')), rdflib.term.Literal('0.74001455', datatype=rdflib.term.URIRef('http://www.w3.org/2001/XMLSchema#float')), rdflib.term.Literal('0.014921662', datatype=rdflib.term.URIRef('http://www.w3.org/2001/XMLSchema#float'))), (rdflib.term.URIRef('http://imitrob.ciirc.cvut.cz/ontologies/crow#cube_holes_od_732'), rdflib.term.Literal('0.6685969', datatype=rdflib.term.URIRef('http://www.w3.org/2001/XMLSchema#float')), rdflib.term.Literal('0.49664876', datatype=rdflib.term.URIRef('http://www.w3.org/2001/XMLSchema#float')), rdflib.term.Literal('0.046712816', datatype=rdflib.term.URIRef('http://www.w3.org/2001/XMLSchema#float')))]



    def check_position_in_workspace_area(self, xyz_list):
        """
        Check position xyz_list=[x,y,z] in area with name 'workspace'
        """
        areas_uris = self.getStoragesProps()
        for area_uri in areas_uris:
            if area_uri['name'] == 'workspace':
                area_poly = self.get_polyhedron(area_uri['uri'])
                return test_in_hull(xyz_list, area_poly)

        self.__node.get_logger().info("<crowracle_client.py> 'workspace' scene doesn't exist!")
        return False

    def pair_objects_to_areas(self, areas_uris, verbose=False):
        """Test if objects are located inside any area, if so - append that object
        to that area

        Args:
            area_uris (URIRef): list of URI's of all the areas in workspace
            verbose (bool): If verbose is set to True, write out all triples who have
                predicate self.CROW.insideOf
        Returns:
            -
        """
        # Get all scene objects
        scene_objs_uris = self.getTangibleObjects()
        for scene_obj_uri in scene_objs_uris:
            # Remove all previous occurences and if objects is within some area -
            # append it to it
            self.onto.remove((scene_obj_uri, self.CROW.insideOf, None))

            for area_uri in areas_uris:
                obj_scene_test_ret = self.test_obj_in_area(obj_uri=scene_obj_uri, area_uri=area_uri)

                if obj_scene_test_ret:
                    self.onto.add((scene_obj_uri, self.CROW.insideOf, area_uri))

        if verbose:
            print("* All objects in all spaces:")
            all = self.onto.triples((None, self.CROW.insideOf, None))
            i = 1
            for bit in all:
                print(f"#{i} triple: {bit}")
                i += 1

        return

    def get_objs_in_area(self, area_uri):
        """Return objects located inside the given area (defined as storage space)

        Args:
            area_uri (URIRef): URI of the storage space

        Returns:
            objs_in (list of URIRefs): URIs of objects in the area
        """
        objs_all = self.getTangibleObjects()
        objs_in = []
        for obj_uri in objs_all:
            res = self.test_obj_in_area(obj_uri, area_uri)
            if res:
                objs_in.append(obj_uri)
        return objs_in

    def get_free_space_area(self, area_uri, spacing=0.05):
        """Return location of free (the least filled) space inside the given area (defined as storage space)

        Args:
            area_uri (URIRef): URI of the storage space
            spacing (float): discretization of locations in area

        Returns:
            free_space_coordinates (list of floats): xyz of the free (the least filled) space in the area
        """
        objs_in = self.get_objs_in_area(area_uri)
        objs_location = []
        for obj_uri in objs_in:
            res = self.get_location_of_obj(obj_uri)
            if res:
                objs_location.append(res)
        polygon = np.asarray(self.get_polygon(area_uri))
        z_mean = np.mean(polygon[:,2])
        x_lim = [min(polygon[:,0]), max(polygon[:,0])]
        y_lim = [min(polygon[:,1]), max(polygon[:,1])]
        x_loc = np.arange(x_lim[0] + spacing, x_lim[-1] - spacing, spacing)
        y_loc = np.arange(y_lim[0] + spacing, y_lim[-1] - spacing, spacing)
        area_location = []
        min_dist_to_objs = []
        for x in x_loc:
            for y in y_loc:
                area_location.append([x, y])
                dist = []
                for obj in objs_location:
                    dist.append(np.linalg.norm(np.asarray(obj[:2]) - np.asarray([x, y])))
                min_dist_to_objs.append(min(dist))
        free_space = area_location[np.argmax(np.asarray(min_dist_to_objs))]
        return [free_space[0], free_space[1], z_mean + spacing]

    def add_storage_space(self, name, polygon, polyhedron, area, volume, centroid):
        """
        Add new storage space defined by markers

        Args:
            name (str): name of storage space
            polygon (list of lists): 3d points defining the base of the storage space
            polyhedron (list of lists): all 3d points defininf the storage space
            area (float): area of the base polygon (base of the storage space)
            volume (float): volume of the polyhedron (storage space)
            centroid (list): location of the storage space (the base)
        """
        self.__node.get_logger().info("CREATING storage {}, location: [{:.2f},{:.2f},{:.2f}].".format(name, *centroid))
        storage_uuid = str(uuid4()).replace("-", "_")
        norm_name = name.replace(" ", "_")
        norm_name = normalize('NFKD', norm_name).encode('ascii', 'ignore').decode("utf-8")
        onto_name = self.CROW[norm_name]
        PART = Namespace(f"{ONTO_IRI}/{norm_name}#") #ns for each storage space
        self.onto.add((onto_name, RDF.type, self.CROW.StorageSpace))
        self.onto.add((onto_name, self.CROW.hasName, Literal(name, datatype=XSD.string)))
        self.onto.add((onto_name, self.CROW.hasUuid, Literal(storage_uuid, datatype=XSD.string)))
        self.onto.add((onto_name, self.CROW.hasArea, Literal(area, datatype=XSD.float)))
        self.onto.add((onto_name, self.CROW.hasVolume, Literal(volume, datatype=XSD.float)))
        self.onto.add((onto_name, self.CROW.isActive, Literal(True, datatype=XSD.boolean)))

        onto_location = PART['xyzAbsoluteLocation']
        self.onto.add((onto_location, RDF.type, self.CROW.Location))
        self.onto.add((onto_location, self.CROW.x, Literal(centroid[0], datatype=XSD.float)))
        self.onto.add((onto_location, self.CROW.y, Literal(centroid[1], datatype=XSD.float)))
        self.onto.add((onto_location, self.CROW.z, Literal(centroid[2], datatype=XSD.float)))
        self.onto.add((onto_name, self.CROW.hasAbsoluteLocation, onto_location))

        onto_polygon = PART['Polygon']
        self.onto.add((onto_polygon, RDF.type, self.CROW.Vector))
        for idx, point in enumerate(polygon):
            point_name = PART['PolygonPoint{}'.format(idx)]
            self.onto.add((point_name, RDF.type, self.CROW.Point3D))
            self.onto.add((point_name, self.CROW.x, Literal(point[0], datatype=XSD.float)))
            self.onto.add((point_name, self.CROW.y, Literal(point[1], datatype=XSD.float)))
            self.onto.add((point_name, self.CROW.z, Literal(point[2], datatype=XSD.float)))
            self.onto.add((onto_polygon, self.CROW.hasPoint3D, point_name))
        self.onto.add((onto_name, self.CROW.hasPolygon, onto_polygon))

        onto_polyhedron = PART['Polyhedron']
        self.onto.add((onto_polyhedron, RDF.type, self.CROW.Vector))
        for idx, point in enumerate(polyhedron):
            point_name = PART['PolyhedronPoint{}'.format(idx)]
            self.onto.add((point_name, RDF.type, self.CROW.Point3D))
            self.onto.add((point_name, self.CROW.x, Literal(point[0], datatype=XSD.float)))
            self.onto.add((point_name, self.CROW.y, Literal(point[1], datatype=XSD.float)))
            self.onto.add((point_name, self.CROW.z, Literal(point[2], datatype=XSD.float)))
            self.onto.add((onto_polyhedron, self.CROW.hasPoint3D, point_name))
        self.onto.add((onto_name, self.CROW.hasPolyhedron, onto_polyhedron))

    def add_storage_space_flat(self, name, polygon, centroid=None, height=0.3, isMainArea=False):
        """
        Add new storage space. Without the need to specify polyhedron, area, and volume.
        If centroid is None, mean of the polygon is used as centroid.

        Args:
            name (str): name of storage space
            polygon (list of lists): 2d points defining the base of the storage space (assumes "shape" = (Nx2) or (Nx3))
            centroid (list): location of the storage space (the base)
            height (float): height of the storage space (polyhedron)
        """
        storage_uuid = str(uuid4()).replace("-", "_")
        norm_name = name.replace(" ", "_")
        norm_name = normalize('NFKD', norm_name).encode('ascii', 'ignore').decode("utf-8")
        onto_name = self.CROW[norm_name]
        PART = Namespace(f"{ONTO_IRI}/{norm_name}#") #ns for each storage space
        onto_location = PART['xyzAbsoluteLocation'].n3()
        onto_polygon = PART['Polygon'].n3()
        onto_polyhedron = PART['Polyhedron'].n3()

        if type(polygon) is list:
            polygon = np.array(polygon, dtype=float)
        if polygon.shape[1] == 2:
            polygon = np.hstack((polygon, np.zeros((polygon.shape[0], 1))))
        # compute the centroid
        if centroid is None:
            centroid = np.mean(polygon, axis=0)
        # compute the area
        n = len(polygon)
        area = []
        for i in range(1,n-1):
            area.append(0.5 * np.cross(polygon[i] - polygon[0], polygon[(i+1)%n] - polygon[0]))
        area = np.asarray(area).sum(axis = 0)
        area = np.linalg.norm(area)
        # "extrude" polygon to polyhedron
        polyhedron = np.vstack((polygon, polygon + np.r_[0, 0, height]))
        # compute the volume of the polyhedron
        volume = area * height

        self.__node.get_logger().info("CREATING storage {}, location: [{:.2f},{:.2f},{:.2f}].".format(name, *centroid))
        query = """PREFIX rdf: <http://www.w3.org/1999/02/22-rdf-syntax-ns#>
            prefix owl: <http://www.w3.org/2002/07/owl#>
            prefix rdfs: <http://www.w3.org/2000/01/rdf-schema#>
            prefix crow: <http://imitrob.ciirc.cvut.cz/ontologies/crow#>

            INSERT DATA {{
                {individual} a crow:StorageSpace .
                {individual} crow:hasName {name} .
                {individual} crow:hasUuid {uuid} .
                {individual} crow:isActive {active} .
                {individual} crow:hasArea {area} .
                {individual} crow:hasVolume {volume} .

                {loc_name} a crow:Location .
                {loc_name} crow:x {loc_x} .
                {loc_name} crow:y {loc_y} .
                {loc_name} crow:z {loc_z} .
                {individual} crow:hasAbsoluteLocation {loc_name} .

                {onto_polygon} a crow:Vector .
                {individual} crow:hasPolygon {onto_polygon} .
                {onto_polyhedron} a crow:Vector .
                {individual} crow:hasPolyhedron {onto_polyhedron} .

            """.format(**{
                "individual": onto_name.n3(),
                "name": Literal(name, datatype=XSD.string).n3(),
                "uuid": Literal(storage_uuid, datatype=XSD.string).n3(),
                "active": Literal(True, datatype=XSD.boolean).n3(),
                "area": Literal(area, datatype=XSD.float).n3(),
                "volume": Literal(volume, datatype=XSD.float).n3(),
                "loc_name": onto_location,
                "loc_x": Literal(centroid[0], datatype=XSD.float).n3(),
                "loc_y": Literal(centroid[1], datatype=XSD.float).n3(),
                "loc_z": Literal(centroid[2], datatype=XSD.float).n3(),
                "onto_polygon": onto_polygon,
                "onto_polyhedron": onto_polyhedron,
            })

        if isMainArea:
            query += f"{onto_name.n3()} crow:areaType 'main' .\n"

        for idx, point in enumerate(polygon):
            point_name = PART['PolygonPoint{}'.format(idx)].n3()
            query += f"{point_name} a crow:Point3D .\n"
            query += f"{point_name} crow:x {Literal(point[0], datatype=XSD.float).n3()} .\n"
            query += f"{point_name} crow:y {Literal(point[1], datatype=XSD.float).n3()} .\n"
            query += f"{point_name} crow:z {Literal(point[2], datatype=XSD.float).n3()} .\n"
            query += f"{onto_polygon} crow:hasPoint3D {point_name} .\n"

        for idx, point in enumerate(polyhedron):
            point_name = PART['PolyhedronPoint{}'.format(idx)].n3()
            query += f"{point_name} a crow:Point3D .\n"
            query += f"{point_name} crow:x {Literal(point[0], datatype=XSD.float).n3()} .\n"
            query += f"{point_name} crow:y {Literal(point[1], datatype=XSD.float).n3()} .\n"
            query += f"{point_name} crow:z {Literal(point[2], datatype=XSD.float).n3()} .\n"
            query += f"{onto_polyhedron} crow:hasPoint3D {point_name} .\n"

        query += "\n}" # add the ending bracket
        # print(query)

        self.onto.update(query)

    def add_position(self, name, centroid):
        """
        Add new position defined by markers

        Args:
            name (str): name of position
            centroid (list): location of the position
        """
        self.__node.get_logger().info("CREATING position {}, location: [{:.2f},{:.2f},{:.2f}].".format(name, *centroid))
        position_uuid = str(uuid4()).replace("-", "_")
        norm_name = name.replace(" ", "_")
        norm_name = normalize('NFKD', norm_name).encode('ascii', 'ignore').decode("utf-8")
        onto_name = self.CROW[norm_name]
        PART = Namespace(f"{ONTO_IRI}/{norm_name}#") #ns for each position
        onto_location = PART['xyzAbsoluteLocation']
        query = """PREFIX rdf: <http://www.w3.org/1999/02/22-rdf-syntax-ns#>
            prefix owl: <http://www.w3.org/2002/07/owl#>
            prefix rdfs: <http://www.w3.org/2000/01/rdf-schema#>
            prefix crow: <http://imitrob.ciirc.cvut.cz/ontologies/crow#>

            INSERT DATA {{
                {individual} a crow:Position .
                {individual} crow:hasName {name} .
                {individual} crow:hasUuid {uuid} .
                {individual} crow:isActive {active} .

                {loc_name} a crow:Location .
                {loc_name} crow:x {loc_x} .
                {loc_name} crow:y {loc_y} .
                {loc_name} crow:z {loc_z} .
                {individual} crow:hasAbsoluteLocation {loc_name} .
            }}
            """.format(**{
                "individual": onto_name.n3(),
                "name": Literal(name, datatype=XSD.string).n3(),
                "uuid": Literal(position_uuid, datatype=XSD.string).n3(),
                "active": Literal(True, datatype=XSD.boolean).n3(),
                "loc_name": onto_location.n3(),
                "loc_x": Literal(centroid[0], datatype=XSD.float).n3(),
                "loc_y": Literal(centroid[1], datatype=XSD.float).n3(),
                "loc_z": Literal(centroid[2], datatype=XSD.float).n3(),
            })
        self.onto.update(query)
        # self.onto.update(self.prepareQuery(query))

    def update_current_action(self, action_name, time):
        """
        Update current detected action and info about the action after a detection comes

        Args:
            action_name (str): name of the current action (action detector name)
            time (str): timestamp of the last action detection, in XSD.dateTimeStamp format
        """
        self.__node.get_logger().info("UPDATING CurrentAction: {}, time: {}.".format(action_name, time))

        # Add action and its properties
        self.onto.set((self.CROW.CurrentAction, self.CROW.hasName, Literal(action_name, datatype=XSD.string)))
        self.onto.set((self.CROW.CurrentAction, self.CROW.hasStopTimestamp, Literal(time, datatype=XSD.dateTimeStamp)))

    def add_detected_action(self, action_name, start, stop, adder_id):
        """
        Add detected action and info about the action after a detection comes

        Args:
            action_name (str): name of the action to be added (action detector name)
            start (str): timestamp of the beginning of the action, in XSD.dateTimeStamp format
            stop (str): timestamp of the end of the action, in XSD.dateTimeStamp format
            adder_id (str): id of action given by adder node, according to the amount and order of overall action detections
        """
        individual_name = action_name.replace(" ", "_") + '_ad_'+str(adder_id)
        self.__node.get_logger().info("ADDING action {}, start: {}, end: {}.".format(individual_name, start, stop))

        # Add action and its properties
        self.onto.add((self.CROW[individual_name], RDF.type, self.CROW.Action))
        self.onto.add((self.CROW[individual_name], self.CROW.hasName, Literal(action_name, datatype=XSD.string)))
        self.onto.add((self.CROW[individual_name], self.CROW.hasUuid, Literal(str(uuid4()).replace("-", "_"), datatype=XSD.string)))
        self.onto.add((self.CROW[individual_name], self.CROW.hasStartTimestamp, Literal(start, datatype=XSD.dateTimeStamp)))
        self.onto.add((self.CROW[individual_name], self.CROW.hasStopTimestamp, Literal(stop, datatype=XSD.dateTimeStamp)))

    def update_object(self, object, location, size, timestamp):
        self.onto.update(self._query_update_object, initBindings={
            "individual": object,
            "new_stamp": Literal(timestamp, datatype=XSD.dateTimeStamp),
            "new_x": Literal(location[0], datatype=XSD.float),
            "new_y": Literal(location[1], datatype=XSD.float),
            "new_z": Literal(location[2], datatype=XSD.float),
            "new_pcl_x": Literal(size[0], datatype=XSD.float),
            "new_pcl_y": Literal(size[1], datatype=XSD.float),
            "new_pcl_z": Literal(size[2], datatype=XSD.float),
        })

    def update_detected_object(self, object, location, size, uuid, timestamp):
        """
        Update info about an existing object after new detection for this object comes

        Args:
            object (URIRef): existing object to be updated
            location (list of floats): new xyz of object's location received from detection
            size (list of floats): new xyz dimensions of object's pointcloud received from detection
            timestamp (str): timestamp of new detection of object, in XSD.dateTimeStamp format
        """
        if self.lock.acquire(1):
            individual_name = object.split('#')[-1]
            self.__node.get_logger().info("UPDATING object {}, timestamp: {}, location: [{:.2f},{:.2f},{:.2f}].".format(individual_name, timestamp, *location))
            self.onto.set((object, self.CROW.hasTimestamp, Literal(timestamp, datatype=XSD.dateTimeStamp)))
            self.onto.set((object, self.CROW.hasUuid, Literal(uuid, datatype=XSD.string)))

            abs_loc = list(self.onto.objects(self.CROW[individual_name], self.CROW.hasAbsoluteLocation))
            if len(abs_loc) > 0:
                self.onto.set((abs_loc[0], self.CROW.x, Literal(location[0], datatype=XSD.float)))
                self.onto.set((abs_loc[0], self.CROW.y, Literal(location[1], datatype=XSD.float)))
                self.onto.set((abs_loc[0], self.CROW.z, Literal(location[2], datatype=XSD.float)))
            else:
                self.__node.get_logger().info("Object {} location update failed.".format(individual_name))

            pcl_dim = list(self.onto.objects(self.CROW[individual_name], self.CROW.hasPclDimensions))
            if len(pcl_dim) > 0:
                self.onto.set((pcl_dim[0], self.CROW.x, Literal(size[0], datatype=XSD.float)))
                self.onto.set((pcl_dim[0], self.CROW.y, Literal(size[1], datatype=XSD.float)))
                self.onto.set((pcl_dim[0], self.CROW.z, Literal(size[2], datatype=XSD.float)))

            self.lock.release()

    def add_detected_object_no_template(self, object_name, location, size, uuid, timestamp, adder_id):
        """
        Add newly detected object and info about the object after a detection for this object comes

        Args:
            object_name (str): name of the object to be added (detector name)
            location (list of floats): xyz of object's location received from detection
            size (list of floats): xyz dimensions of object's pointcloud received from detection
            uuid (str): id of object given by filter node (id of corresponding model in the filter)
            timestamp (str): timestamp of new detection of object, in XSD.dateTimeStamp format
            template (URIRef): template object from ontology corresponding to the detected object
            adder_id (str): id of object given by adder node, according to the amount and order of overall object detections
        """
        individual_name = object_name + '_od_'+str(adder_id)
        PART = Namespace(f"{ONTO_IRI}/{individual_name}#")

        initBindings = {
            "individual": self.CROW[individual_name],
            "loc_name": PART.xyzAbsoluteLocation,
            "loc_x": Literal(location[0], datatype=XSD.float),
            "loc_y": Literal(location[1], datatype=XSD.float),
            "loc_z": Literal(location[2], datatype=XSD.float),
            "pcl_name": PART.hasPclDimensions,
            "pcl_x": Literal(size[0], datatype=XSD.float),
            "pcl_y": Literal(size[1], datatype=XSD.float),
            "pcl_z": Literal(size[2], datatype=XSD.float),
            "det_name": Literal(object_name, datatype=XSD.string),
            "adder_id": Literal('od_'+str(adder_id), datatype=XSD.string),
            "uuid": Literal(uuid, datatype=XSD.string),
            "stamp": Literal(timestamp, datatype=XSD.dateTimeStamp)
        }
        self.onto.update(self._query_add_object_no_template, initBindings=initBindings)
        self.node.get_logger().info(f"Added object {individual_name} with uuid {uuid} and id od_{adder_id}")

    def add_detected_object(self, object_name, location, size, uuid, timestamp, template, adder_id):
        """
        Add newly detected object and info about the object after a detection for this object comes

        Args:
            object_name (str): name of the object to be added (detector name)
            location (list of floats): xyz of object's location received from detection
            size (list of floats): xyz dimensions of object's pointcloud received from detection
            uuid (str): id of object given by filter node (id of corresponding model in the filter)
            timestamp (str): timestamp of new detection of object, in XSD.dateTimeStamp format
            template (URIRef): template object from ontology corresponding to the detected object
            adder_id (str): id of object given by adder node, according to the amount and order of overall object detections
        """
        individual_name = object_name + '_od_'+str(adder_id)
        PART = Namespace(f"{ONTO_IRI}/{individual_name}#")

        initBindings = {
            "individual": self.CROW[individual_name],
            "loc_name": PART.xyzAbsoluteLocation,
            "loc_x": Literal(location[0], datatype=XSD.float),
            "loc_y": Literal(location[1], datatype=XSD.float),
            "loc_z": Literal(location[2], datatype=XSD.float),
            "pcl_name": PART.hasPclDimensions,
            "pcl_x": Literal(size[0], datatype=XSD.float),
            "pcl_y": Literal(size[1], datatype=XSD.float),
            "pcl_z": Literal(size[2], datatype=XSD.float),
            "template": template,
            "adder_id": Literal('od_'+str(adder_id), datatype=XSD.string),
            "uuid": Literal(uuid, datatype=XSD.string),
            "stamp": Literal(timestamp, datatype=XSD.dateTimeStamp)
        }
        self.onto.update(self._query_add_object, initBindings=initBindings)

    def get_objects_by_uuid(self, uuids):
        """ Returns a list of object URIs for every UUID that exists in the database.
        """
        if type(uuids) is not list:
            uuids = [uuids]
        query = """PREFIX rdf: <http://www.w3.org/1999/02/22-rdf-syntax-ns#>
        prefix owl: <http://www.w3.org/2002/07/owl#>
        prefix rdfs: <http://www.w3.org/2000/01/rdf-schema#>
        prefix crow: <http://imitrob.ciirc.cvut.cz/ontologies/crow#>

        SELECT DISTINCT ?obj ?uuid

        WHERE {{
            # ?obj a ?cls .
            # ?cls rdfs:subClassOf+ crow:TangibleObject .
            ?obj crow:hasUuid ?uuid .
            FILTER EXISTS {{ ?obj crow:hasTimestamp ?any }}
            FILTER (?uuid IN ({list_of_uuid}))

        }}""".format(list_of_uuid=",".join([f"'{u}'" for u in uuids]))
        result = self.onto.query(self.prepareQuery(query))
        return list(result)

    def delete_object(self, obj):
        """
        Delete existing object and all info about the object

        Args:
            object (URIRef): existing object to be deleated
        """
        self.__node.get_logger().info("DELETING object {}.".format(obj.split('#')[-1]))
        query = """DELETE {{
                ?s ?p ?o .
            }}
            WHERE {{
                ?s ?p ?o .
                FILTER (?s = {individual} || ?p = {individual})
            }}""".format(individual=obj.n3())
        # self.onto.update(self.prepareQuery(query))
        self.onto.update(query)

    def disable_object(self, obj):
        """
        Disable existing object - temporarly remove id

        Args:
            object (URIRef): existing object to be disabled
        """
        # if len(list(self.onto.triples((obj, RDF.type, OWL.NamedIndividual)))) > 0:
        #     id = list(self.onto.objects(obj, self.CROW.hasId))
        #     if len(id) > 0:
        #         self.__node.get_logger().info("DISABLING object {}.".format(obj.split('#')[-1]))
        #         self.onto.remove((obj, self.CROW.hasId, None))
        #         self.onto.add((obj, self.CROW.disabledId, id[0]))
        self.__node.get_logger().info("DISABLING object {}.".format(obj.split('#')[-1]))
        self.onto.update(self._query_disable_object, initBindings={"individual": obj})

    def enable_object(self, obj):
        """
        Enable existing object - refresh temporarly removed id

        Args:
            object (URIRef): existing object to be enabled
        """
        # if len(list(self.onto.triples((obj, RDF.type, OWL.NamedIndividual)))) > 0:
        #     id = list(self.onto.objects(obj, self.CROW.disabledId))
        #     if len(id) > 0:
        #         self.__node.get_logger().info("ENABLING object {}.".format(obj.split('#')[-1]))
        #         self.onto.remove((obj, self.CROW.disabledId, None))
        #         self.onto.add((obj, self.CROW.hasId, id[0]))
        self.__node.get_logger().info("ENABLING object {}.".format(obj.split('#')[-1]))
        self.onto.update(self._query_enable_object, initBindings={"individual": obj})

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
