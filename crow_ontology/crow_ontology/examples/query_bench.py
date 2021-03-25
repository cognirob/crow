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
        res = self.onto.triples((None, self.CROW.hasColor, None))
        return [t for t, _, _, in res]

    def time_function(self, func, r=5, n=10, *args, **kwargs):
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
        # self.print_list(resA)
        # self.print_list(resB)
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

def main():
    rclpy.init()
    ot = OntoTester()
    ot.test_object()


if __name__ == "__main__":
    main()
