from rdflib.namespace import FOAF, RDFS, RDF, OWL, XMLNS, XSD, Namespace
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

    # 7
    def get_location_of_obj(self, uri):
        """Get absolute location of URI object

        Args:
            uri (URIRef): URI of obj, 1

        Returns:
            list of floats: xyz location, 1x3
        """
        loc_obj = list(self.onto.objects(uri, self.CROW.hasAbsoluteLocation))
        if len(loc_obj) > 0: # assume obj has max one location
            try: # expect floats
                loc = [float(list(self.onto.objects(loc_obj[0], x))[0]) for x in [self.CROW.x, self.CROW.y, self.CROW.z]]
            except: # but may be None (if not localized yet)
                loc = [str(list(self.onto.objects(loc_obj[0], x))[0]) for x in [self.CROW.x, self.CROW.y, self.CROW.z]]
            return loc
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
            return color[0] # assume obj has only one color
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
            else:
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
        # nlp names have classes or colors
        if len(list(self.onto.triples((uri, RDF.type, OWL.Class)))) > 0:
            uri = uri # if ent is a class, then proceed
        elif len(list(self.onto.triples((uri, RDF.type, self.CROW.NamedColor)))) > 0:
            uri = uri # if ent is named color, then proceed
        else: # uri is a object, it's class should have nlp name
            uri = list(self.onto.objects(uri, RDF.type))[0]

        if language == 'EN':
            nlp_name_property = self.CROW.hasNlpNameEN
        elif language == 'CZ':
            nlp_name_property = self.CROW.hasNlpNameCZ
        else:
            "Invalid language choice (EN or CZ), taking default EN option"
            nlp_name_property = self.CROW.hasNlpNameEN

        nlp_name_uri = list(self.onto.objects(uri, nlp_name_property))
        if len(nlp_name_uri) < 1: # no nlp name -> create new from the uri string
            nlp_name = [uri.split('#')[-1]]
            self.onto.add((uri, nlp_name_property, Literal(nlp_name[0], datatype=XSD.string)))
        else:
            nlp_name = [x.toPython() for x in nlp_name_uri]
        return nlp_name

    # A "which all objects are in the ontology?"
    def get_all_tangible_nlp(self, language='EN'):
        """Get nlp names of all tangible objects

        Args:
            language (str): nlp names in which language

        Returns:
            list of strings: nlp names of all tangible objects, 0...N
        """
        all_tangible = self.getTangibleObjectClasses()
        all_tangible_nlp = []
        for tangible in all_tangible:
            all_tangible_nlp.append(self.get_nlp_from_uri(tangible[0], language=language))
        return all_tangible_nlp

    # B "which objects are in the scene?"
    def get_tangible_nlp(self, language='EN'):
        """Get nlp names of tangible objects in the scene

        Args:
            language (str): nlp names in which language

        Returns:
            list of strings: nlp names of tangible objects in the scene, 0...N
        """
        all_tangible = self.getTangibleObjects()
        all_tangible_nlp = []
        for tangible in all_tangible:
            all_tangible_nlp.append(self.get_nlp_from_uri(tangible, language=language))
        return all_tangible_nlp

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
            color = self.get_color_of_obj(uri) # assume one obj has one color
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

    # E 
    def get_all_obj_of_color_nlp(self, name, language='EN'): 
        # NOT NEEDED! have 'all' parameter in get_obj_of_color_nlp()
        #same as get_obj_of_color_nlp but only for objects in the scene
        uris = self.get_uri_from_nlp(name)
        obj_names = []
        for uri in uris:
            obj_uris = self.get_obj_of_color(uri)
            for obj_uri in obj_uris:
                obj_names.append(self.get_nlp_from_uri(obj_uri, language=language))
        obj_names = list(set(sum(obj_names, [])))
        return obj_names

    # G "where (in the scene) is cube?"
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
