from rdflib.namespace import FOAF, RDFS, RDF, OWL, XMLNS, Namespace
from rdflib.extras.infixowl import Class
from rdflib import BNode, URIRef, Literal
from knowl import OntologyAPI
from enum import Enum
from importlib.util import find_spec
import os


class State(Enum):
    DEFAULT = 1
    LEARN_FROM_INSTRUCTIONS = 2


class Crowtology():
    """CROW Ontology class handler
    Holds an instance of an ontology API and sets the parameters to necessary to setup the CROW ontology
    """
    crowNSString = "http://www.semanticweb.org/crow/ontologies/2019/6/onto_draft_01#"
    CROW = Namespace(crowNSString)
    OWLR = Namespace("http://www.lesfleursdunormal.fr/static/_downloads/owlready_ontology.owl#")

    def __init__(self, autoMergeIfEmpty=True):
        modulePath = find_spec("crow_ontology").submodule_search_locations[0]
        cfgPath = os.path.join(modulePath, "..", "config", "db_config.yaml")
        self.__onto = OntologyAPI(cfgPath)
        self.__onto.bind("crow", self.CROW)
        if autoMergeIfEmpty and len(self.onto) == 0:
            # TODO: make nicer
            ontologyTemplatePath = os.path.join(modulePath, "..", "..", "ontology", "onto_draft_01.owl")
            self.onto.mergeFileIntoDB(ontologyTemplatePath)

    @property
    def onto(self):
        return self.__onto

    def translateKey(self, key):
        """Translates Pyhon object property name (such as "color") into a proper ontology predicate (e.g. CROW:hasColor)
        that can be used as a predicate in a query to the ontology. This is useful for the OOP implementation
        of the ontology in Python.

        Parameters
        ----------
        key : str
            Python object attribute name that has equivalent in the ontology.

        Returns
        -------
        URIRef
            URI reference to the ontology entity/predicate.
        """
        # TODO: more sophisticated translation
        return self.onto.value(None, self.OWLR.python_name, key)


# class LegacyAPI(Crowtology):

#     def __init__(self):
#         super().__init__()
#         self.params = {}

#         # TODO see DatabaseAPI.add_object()
#         self.objects = []

#         self.set_param("state", State.DEFAULT)


#     def add_object(self, objectClass: str, x : float, y : float, z : float, aruco_id : str = None, color : str = None):
#         """
#         Adds a new object in the ontology.

#         Parameters
#         ----------
#         Class the class of the ontology object to be added, should be a subclass of onto.Object
#         x     the position of the object: x coordinate
#         y     the position of the object: y coordinate
#         id    the identifier of the object, e.g. ArUco id
#         color the color of the object as string, e.g. "red"
#         -------

#         """
#         # TODO: ImplementMe!
#         self.onto.add((BNode(), RDF.type, self.CROW[objectClass]))


#     # TODO this method is intended to be used as an entry point to ROS node, the interface should be changed to work with ROS messages
#     def update_object_position(self, aruco_id : str, x : float, y: float, z: float = 0):
#         """
#         Updates the position of the existing object or adds the object if it does not exist yet.

#         Parameters
#         ----------
#         id  the identifier of the object, e.g. ArUco id
#         x   the position of the object: x coordinate
#         y   the position of the object: y coordinate
#         """
#         # TODO: ImplementMe!
#         pass

#     def delete_object(self, obj : Any):
#         """
#         Deletes the object from the ontology.

#         Parameters
#         ----------
#         obj  the object to be deleted
#         """
#         # ow.destroy_entity(obj)
#         # TODO: ImplementMe!
#         pass

#     def get_class_objects(self, cls: ClassVar) -> List:
#         """
#         Returns a list of real objects in the workspace with a certain class.

#         Parameters
#         ----------
#         cls  the class of the objects
#         """
#         # return onto.search(type=cls, _is_in_workspace=True)
#         # TODO: ImplementMe!
#         pass

#     def get_all_tools(self) -> list:
#         """
#         Returns a list of all tools in the workspace.
#         """
#         # return onto.search(type=onto.Tool, _is_in_workspace=True)
#         # TODO: ImplementMe!
#         pass

#     def get_class_objects_by_color(self, cls: ClassVar, color: str) -> List:
#         """
#         Returns a list of real objects in the workspace with a certain class and a certain color.

#         Parameters
#         ----------
#         cls   the class of the objects
#         color the color of the objects as string
#         """
#         # return onto.search(type=cls, color=onto.NamedColor(color), _is_in_workspace=True)
#         # TODO: ImplementMe!
#         pass

#     def get_objects_by_color(self, color: str) -> List:
#         """
#         Returns a list of real objects in the workspace with a certain color.

#         Parameters
#         ----------
#         color the color of the objects as string
#         """
#         # return onto.search(color=onto.NamedColor(color), _is_in_workspace=True)
#         # TODO: ImplementMe!
#         pass


#     def get_last_mentioned_object(self):
#         """
#         Returns the object which was set as last mentioned or None if there is no such object.
#         """
#         # return onto.search(_is_last_mentioned=True, _is_in_workspace=True).first()
#         # TODO: ImplementMe!
#         pass


#     def set_last_mentioned_object(self, obj):
#         """
#         Sets an object as last mentioned.

#         Parameters
#         ----------
#         obj  the object which should be set as "last mentioned"
#         """
#         # obj._is_last_mentioned = True
#         # TODO: ImplementMe!
#         pass


#     def set_state(self, state : State):
#         """
#         Sets a current state of the program.

#         Parameters
#         ----------
#         state  the state to be set
#         """
#         # db.set_param("state", state)
#         # TODO: ImplementMe!
#         pass


#     def set_param(self, key : str, val):
#         """
#         Saves an arbitrary key-value pair to be retrieved later

#         Parameters
#         ----------
#         key  the key of the parameter
#         val  the value of the parameter
#         """
#         # db.set_param(key, val)
#         # TODO: ImplementMe!
#         pass


#     def get_state(self):
#         """
#         Returns current program state.
#         """
#         # return db.get_param("state")
#         # TODO: ImplementMe!
#         pass

#     def get_custom_templates(self):
#         """
#         Returns a list of custom templates learned during the learning phase.
#         """
#         # return custom_templates
#         # TODO: ImplementMe!
#         pass

#     def get_demonstration_templates(self):
#         """
#         Returns a list of templates learned during the demonstration.
#         """
#         # return demonstration_templates
#         # TODO: ImplementMe!
#         pass

#     def add_custom_template(self, template):
#         """
#         Adds a custom template to the ontology.

#         Parameters
#         ----------
#         template  the template to be added
#         """
#         # TODO: ImplementMe!
#         pass


#     def add_area(self, area_name : str, corners : Tuple[onto.Location, onto.Location, onto.Location, onto.Location], is_default=False):
#         # TODO: ImplementMe!
#         pass


#     def get_default_area(self):
#         # return default_area[0]
#         # TODO: ImplementMe!
#         pass


#     def get_param(self, key : str):
#         """
#         Retrieves the value for the key in the database (previously set by set_param()) or None if the key does not exist.

#         Parameters
#         ----------
#         key  the key of the parameter
#         """
#         # return db.get_param(key)
#         # TODO: ImplementMe!
#         pass


#     def get_by_properties(self, cls : ClassVar, properties : dict):
#         """
#         Returns a list of real objects in the workspace with a certain class and a certain properties
#         (useful for grounding the placeholders).

#         Parameters
#         ----------
#         cls         the class of the objects
#         properties  a dictionary with ontology property (python) names as keys and
#                     ontology objects (from the range of the respective property) as values
#         -------

#         """
#         # return onto.search(type=cls, **properties, _is_in_workspace=True)
#         # TODO: ImplementMe!
#         pass

#     def get_onto(self):
#         """
#         Returns the onto object to be used as a namespace
#         """
#         # TODO: ImplementMe!
#         return None
