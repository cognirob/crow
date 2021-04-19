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
ONTO_IRI = "http://imitrob.ciirc.cvut.cz/ontologies/crow"
OWL_READY = "http://www.lesfleursdunormal.fr/static/_downloads/owlready_ontology.owl"

class CrowtologyClient():

    CROW = Namespace(f"{ONTO_IRI}#")
    OWL_READY_NS = Namespace(f"{OWL_READY}#")

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
    _present_and_disabled_nocls_query = prepareQuery("""SELECT ?obj
        WHERE {
            ?obj crow:hasTimestamp ?c .
        }""",
                                   initNs={"owl": OWL, "crow": CROW}
                                   )
    _present_query_props = prepareQuery("""SELECT ?obj ?id ?cls ?col ?colczname ?colenname ?czname ?enname ?x ?y ?z
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
        }""",
                                   initNs={"owl": OWL, "crow": CROW, "rdf": RDF, "rdfs": RDFS}
                                   )
    _present_nocls_query = prepareQuery("""SELECT ?obj
        WHERE {
            ?obj crow:hasId ?c .
        }""",
                                   initNs={"owl": OWL, "crow": CROW}
                                   )
    _colors_query = prepareQuery("""SELECT ?obj
        WHERE {
            ?obj rdf:type crow:NamedColor .
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
        return [g[0] for g in qres]

    def getTangibleObjects(self):
        """Lists physical objects present on the workspace

        Returns:
            list: The objects.
        """
        res = self.onto.query(self._present_query)
        return [g["obj"] for g in res]

    def getTangibleObjectsProps(self):
        res = self.onto.query(self._present_query_props)
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
        res = self.onto.query(self._present_nocls_query)
        return [g["obj"] for g in res]

    def getTangibleObjects_disabled_nocls(self):
        """Lists physical objects present or disabled on the workspace NO CLS

        Returns:
            list: The objects.
        """
        res = self.onto.query(self._present_and_disabled_nocls_query)
        return [g["obj"] for g in res]

    def getColors(self):
        """Lists all colors in the database

        Returns:
            list: The colors (URIRefs).
        """
        res = self.onto.query(self._colors_query)
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

        q = prepareQuery(q_string, initNs={"rdf": RDF, "crow":self.CROW})
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

    def get_pcl_dimensions_of_obj(self, uri):
        """Get dimensions of pcl of detected object

        Args:
            uri (URIRef): URI of obj, 1

        Returns:
            list of floats: xyz dimension, 1x3
        """
        dim_obj = list(self.onto.objects(uri, self.CROW.hasPclDimensions))
        if len(dim_obj) > 0: # assume obj has max one dimensions
            try: # expect floats
                dim = [float(list(self.onto.objects(dim_obj[0], x))[0]) for x in [self.CROW.x, self.CROW.y, self.CROW.z]]
            except: # but may be None (if not localized yet)
                dim = [str(list(self.onto.objects(dim_obj[0], x))[0]) for x in [self.CROW.x, self.CROW.y, self.CROW.z]]
            return dim
        else:
            return [None]*3

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
        # nlp names have classes or colors
        if len(list(self.onto.triples((uri, RDF.type, OWL.Class)))) > 0:
            uri = uri # if ent is a class, then proceed
        elif len(list(self.onto.triples((uri, RDF.type, self.CROW.NamedColor)))) > 0:
            uri = uri # if ent is named color, then proceed
        else: # uri is a object, it's class should have nlp name
            class_uri = list(self.onto.objects(uri, RDF.type))
            if len(class_uri) > 0:
                uri = class_uri[0]
            else:
                uri = uri # ent is not class nor color, assuming it's an object but didn't find it's class

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
            all_tangible_nlp.append(self.get_nlp_from_uri(tangible, language=language))
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

    def get_colors_nlp(self, language='EN'):
        """Get nlp names of all colors in the database

        Args:
            language (str): nlp names in which language

        Returns:
            list of strings: nlp names of colors in the database, 0...N
        """
        all_colors = self.getColors()
        all_colors_nlp = []
        for color in all_colors:
            all_colors_nlp.append(self.get_nlp_from_uri(color, language=language))
        return all_colors_nlp

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
        q = prepareQuery(q_string, initNs={"rdf": RDF, "rdfs": RDFS, "crow":self.CROW})
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
        new_ns = Namespace(f"{str.split('#')[0]}#")
        obj_uri = new_ns[str.split('#')[-1]]
        return obj_uri

    def update_detected_object(self, object, location, size, timestamp):
        """
        Update info about an existing object after new detection for this object comes

        Args:
            object (URIRef): existing object to be updated
            location (list of floats): new xyz of object's location received from detection
            size (list of floats): new xyz dimensions of object's pointcloud received from detection
            timestamp (str): timestamp of new detection of object, in XSD.dateTimeStamp format
        """
        individual_name = object.split('#')[-1]
        self.__node.get_logger().info("UPDATING object {}, timestamp: {}, location: [{:.2f},{:.2f},{:.2f}].".format(individual_name, timestamp, *location))
        self.onto.set((object, self.CROW.hasTimestamp, Literal(timestamp, datatype=XSD.dateTimeStamp)))
        
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
            adder_id (str): if of object given by adder node, according to the amount and order of overall detections
        """
    
        # Find template object
        all_props = list(self.onto.triples((template, None, None)))
        individual_name = object_name + '_od_'+str(adder_id)
        PART = Namespace(f"{ONTO_IRI}/{individual_name}#") #ns for each object (/cube_holes_1#)
        self.__node.get_logger().info("ADDING object {}, timestamp: {}, location: [{:.2f},{:.2f},{:.2f}].".format(individual_name, timestamp, *location))

        # Add common object properties
        for prop in all_props:
            #add idividual_name_ns#hole1 for all objectParts of template
            if prop[1] == self.CROW.hasObjectPart:
                all_object_part_props = list(self.onto.triples((prop[2], None, None)))
                prop_name = PART[str(prop[2]).split('#')[-1]]
                self.onto.add((self.CROW[individual_name], prop[1], prop_name))
                for object_part_prop in all_object_part_props:
                    self.onto.add((prop_name, object_part_prop[1], object_part_prop[2]))
            #add other properties based on template
            else:
                self.onto.add((self.CROW[individual_name], prop[1], prop[2]))
        # correct references between holes in property 'extendsTo'
        all_object_parts = list(self.onto.objects(self.CROW[individual_name], self.CROW.hasObjectPart))
        for object_part in all_object_parts:
            extendsto_obj = list(self.onto.objects(object_part, self.CROW.extendsTo))
            if len(extendsto_obj) > 0:
                correct_obj = PART[str(extendsto_obj[0]).split('#')[-1]]
                self.onto.set((object_part, self.CROW.extendsTo, correct_obj))

        # Add AbsoluteLocaton (object specific)
        prop_name = PART.xyzAbsoluteLocation
        prop_range = list(self.onto.objects(self.CROW.hasAbsoluteLocation, RDFS.range))[0]
        self.onto.add((prop_name, RDF.type, prop_range))
        self.onto.add((prop_name, self.CROW.x, Literal(location[0], datatype=XSD.float)))
        self.onto.add((prop_name, self.CROW.y, Literal(location[1], datatype=XSD.float)))
        self.onto.add((prop_name, self.CROW.z, Literal(location[2], datatype=XSD.float)))
        self.onto.set((self.CROW[individual_name], self.CROW.hasAbsoluteLocation, prop_name))

        # Add PclDimensions (object specific)
        prop_name = PART.xyzPclDimensions
        prop_range = list(self.onto.objects(self.CROW.hasPclDimensions, RDFS.range))[0]
        self.onto.add((prop_name, RDF.type, prop_range))
        self.onto.add((prop_name, self.CROW.x, Literal(size[0], datatype=XSD.float)))
        self.onto.add((prop_name, self.CROW.y, Literal(size[1], datatype=XSD.float)))
        self.onto.add((prop_name, self.CROW.z, Literal(size[2], datatype=XSD.float)))
        self.onto.set((self.CROW[individual_name], self.CROW.hasPclDimensions, prop_name))

        # Add unique ID and timestamp
        self.onto.add((self.CROW[individual_name], self.CROW.hasId, Literal('od_'+str(adder_id), datatype=XSD.string)))
        self.onto.add((self.CROW[individual_name], self.CROW.hasUuid, Literal(uuid, datatype=XSD.string)))
        self.onto.add((self.CROW[individual_name], self.CROW.hasTimestamp, Literal(timestamp, datatype=XSD.dateTimeStamp)))

    def delete_object(self, obj):
        """
        Delete existing object and all info about the object

        Args:
            object (URIRef): existing object to be deleated
        """
        if len(list(self.onto.triples((obj, RDF.type, OWL.NamedIndividual)))) > 0:
            self.__node.get_logger().info("DELETING object {}.".format(obj.split('#')[-1]))
            self.onto.remove((obj, None, None))
    
    def disable_object(self, obj):
        """
        Disable existing object - temporarly remove id

        Args:
            object (URIRef): existing object to be disabled
        """
        if len(list(self.onto.triples((obj, RDF.type, OWL.NamedIndividual)))) > 0:
            id = list(self.onto.objects(obj, self.CROW.hasId))
            if len(id) > 0:
                self.__node.get_logger().info("DISABLING object {}.".format(obj.split('#')[-1]))
                self.onto.remove((obj, self.CROW.hasId, None))
                self.onto.add((obj, self.CROW.disabledId, id[0]))

    def enable_object(self, obj):
        """
        Enable existing object - refresh temporarly removed id

        Args:
            object (URIRef): existing object to be enabled
        """
        if len(list(self.onto.triples((obj, RDF.type, OWL.NamedIndividual)))) > 0:
            id = list(self.onto.objects(obj, self.CROW.disabledId))
            if len(id) > 0:
                self.__node.get_logger().info("ENABLING object {}.".format(obj.split('#')[-1]))
                self.onto.remove((obj, self.CROW.disabledId, None))
                self.onto.add((obj, self.CROW.hasId, id[0]))
            

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
