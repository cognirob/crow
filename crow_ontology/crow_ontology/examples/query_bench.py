import rclpy
from rclpy.node import Node
import curses
from curses.textpad import Textbox, rectangle
from crow_ontology.crowracle_client import CrowtologyClient
from rdflib.namespace import Namespace, RDF, RDFS, OWL, FOAF
from rdflib import URIRef, BNode, Literal, Graph
from rdflib.term import Identifier
import time
from timeit import repeat
from rdflib.plugins.sparql import prepareQuery
import numpy as np


class OntoTester(Node):
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
            ?obj crow:hasColor ?c .
            ?obj rdf:type ?cls .
            ?cls rdfs:subClassOf* crow:TangibleObject .
        }""",
                                initNs={"owl": OWL, "crow": CROW}
                                )

    _present_nocls_query = prepareQuery("""SELECT ?obj
        WHERE {
            ?obj crow:hasColor ?c .
        }""",
                                initNs={"owl": OWL, "crow": CROW}
                                )

    def __init__(self, node_name="onto_tester"):
        super().__init__(node_name)
        self.crowracle = CrowtologyClient(node=self)
        self.onto = self.crowracle.onto
        # self.get_logger().info(self.onto)

    def getTangibleObjectClasses(self, mustBeLeaf=True):
        """Return classes of all TangibleObjects (i.e. physical objects that can be present on the workspace)

        Args:
            mustBeLeaf (bool, optional): If True, only leaf classes are returned. That is,
            no general classes (e.g. "Workpice", "Tool") will be returned. Defaults to True.

        Returns:
            list: List of RDFLib terms describing the classes. Use str(result[i]) to turn into string.
        """
        qres = self.onto.query(self._tangible_leaf_query if mustBeLeaf else self._tangible_query)
        return qres

    def getTangibleObjects_sparql(self):
        res = self.onto.query(self._present_query)
        return [g["obj"] for g in res]

    def getTangibleObjects_triples(self):
        res = self.onto.triples((None, self.CROW.hasColor, None))
        objects = []
        for tangible, _, id in res:
            for _, _, tcls in self.onto.triples((tangible, RDF.type, None)):
                if self.CROW.TangibleObject in self.onto.transitive_objects(tcls, RDFS.subClassOf):
                    objects.append(tangible)
                    break
        return objects

    def getTangibleObjects_nocls_sparql(self):
        res = self.onto.query(self._present_nocls_query)
        return [g["obj"] for g in res]

    def getTangibleObjects_nocls_triples(self):
        res = self.onto.subjects(self.CROW.hasColor, None)
        return list(res)

    def time_function(self, func, r=3, n=5, *args, **kwargs):
        self.get_logger().info(f"Testing function {func.__name__}:")
        if len(args) > 0 or len(kwargs) > 0:
            func = lambda f=func, a=args, k=kwargs: f(*a, **k)
        ret = np.array(repeat(func, repeat=r, number=n)) / n
        print(f"\tmin time = {min(ret)}\n\tmax time = {max(ret)}\n\tmedian time = {np.median(ret)}")
        return ret

    def print_list(self, ls):
        for e in ls:
            print(type(e))
            print(e)

    def compare_functions(self, funcA, funcB):
        self.get_logger().info("===============>")
        self.get_logger().info(f"Comparing functions {str([f.__name__ for f in [funcA, funcB]])}")
        self.get_logger().info("Testing if outputs are valid...")
        resA = funcA()
        resB = funcB()
        self.print_list(resA)
        self.get_logger().info(".")
        self.print_list(resB)
        assert sorted(resA) == sorted(resB), "The results of the two function differ!"
        self.get_logger().info("OK.")
        self.get_logger().info("Testing runtimes...")
        self.time_function(funcA)
        self.time_function(funcB)
        self.get_logger().info("Done.")
        self.get_logger().info("<===============")

    def test_object(self):
        self.compare_functions(self.getTangibleObjects_sparql, self.getTangibleObjects_triples)
        self.compare_functions(self.getTangibleObjects_nocls_sparql, self.getTangibleObjects_nocls_triples)

    def add_detected_object(self, object_name, location, size, timestamp):
        prop_range = next(self.onto.objects(subject=CROW.hasDetectorName, predicate=RDFS.range))
        corresponding_objects = list(self.onto.subjects(CROW.hasDetectorName, Literal(object_name, datatype=prop_range)))
        self.get_logger().info("Adding detected object {}, id {} at location {}.".format(object_name, 'od_'+str(self.id), location))
        # Find template object
        all_props = list(self.onto.triples((template, None, None)))
        template_type = list(self.onto.objects(template, RDF.type))
        template_type = [x for x in template_type if ONTO_IRI in x][0]
        num_subjects = len(list(self.onto.subjects(RDF.type, template_type)))
        individual_name = object_name + '_' + str(num_subjects)
        PART = Namespace(f"{ONTO_IRI}/{individual_name}#") #ns for each object (/cube_holes_1#)

        # Add common object properties
        for prop in all_props:
            #add idividual_name_ns#hole1 for all objectParts of template
            if prop[1] == CROW.hasObjectPart:
                all_object_part_props = list(self.onto.triples((prop[2], None, None)))
                prop_name = PART[str(prop[2]).split('#')[-1]]
                self.onto.add((CROW[individual_name], prop[1], prop_name))
                for object_part_prop in all_object_part_props:
                    self.onto.add((prop_name, object_part_prop[1], object_part_prop[2]))
            #add other properties based on template
            else:
                self.onto.add((CROW[individual_name], prop[1], prop[2]))
        # correct references between holes in property 'extendsTo'
        all_object_parts = list(self.onto.objects(CROW[individual_name], CROW.hasObjectPart))
        for object_part in all_object_parts:
            extendsto_obj = list(self.onto.objects(object_part, CROW.extendsTo))
            if len(extendsto_obj) > 0:
                correct_obj = PART[str(extendsto_obj[0]).split('#')[-1]]
                self.onto.set((object_part, CROW.extendsTo, correct_obj))

        # Add AbsoluteLocaton (object specific)
        prop_name = PART.xyzAbsoluteLocation
        prop_range = list(self.onto.objects(CROW.hasAbsoluteLocation, RDFS.range))[0]
        self.onto.add((prop_name, RDF.type, prop_range))
        self.onto.add((prop_name, CROW.x, Literal(location[0], datatype=XSD.float)))
        self.onto.add((prop_name, CROW.y, Literal(location[1], datatype=XSD.float)))
        self.onto.add((prop_name, CROW.z, Literal(location[2], datatype=XSD.float)))
        self.onto.set((CROW[individual_name], CROW.hasAbsoluteLocation, prop_name))

        # Add PclDimensions (object specific)
        prop_name = PART.xyzPclDimensions
        prop_range = list(self.onto.objects(CROW.hasPclDimensions, RDFS.range))[0]
        self.onto.add((prop_name, RDF.type, prop_range))
        self.onto.add((prop_name, CROW.x, Literal(size[0], datatype=XSD.float)))
        self.onto.add((prop_name, CROW.y, Literal(size[1], datatype=XSD.float)))
        self.onto.add((prop_name, CROW.z, Literal(size[2], datatype=XSD.float)))
        self.onto.set((CROW[individual_name], CROW.hasPclDimensions, prop_name))

        # Add unique ID and timestamp
        self.onto.add((CROW[individual_name], CROW.hasId, Literal('od_'+str(self.id), datatype=XSD.string)))
        self.id += 1
        self.onto.add((CROW[individual_name], CROW.hasTimestamp, Literal(timestamp, datatype=XSD.dateTimeStamp)))

        # if len(individual_names) == 0:
        #     T_ref2world = []
        #     #first object defines reference coordinate frame
        #     #rel_loc = old one
        # else:
        #     rel_loc = []
        return individual_name

def main():
    rclpy.init()
    ot = OntoTester()
    ot.test_object()


if __name__ == "__main__":
    main()
