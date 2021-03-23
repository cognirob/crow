import rclpy
from rclpy.node import Node
import curses
from curses.textpad import Textbox, rectangle
from crow_ontology.crowracle_client import CrowtologyClient
from rdflib.namespace import Namespace, RDF, RDFS, OWL, FOAF
from rdflib import URIRef, BNode, Literal, Graph
from rdflib.term import Identifier
import time


class OntoTester(Node):

    def __init__(self, node_name="onto_tester"):
        super().__init__(node_name)
        self.crowracle = CrowtologyClient(node=self)
        self.onto = self.crowracle.onto
        # self.get_logger().info(self.onto)

    def test_object(self):
        self.get_logger().info("Found these classes of tangible objects")
        start = time.time()
        qres = self.crowracle.getTangibleObjectClasses()
        for c in qres:
            self.get_logger().info(f"{c}")
        print(time.time() - start)

        start = time.time()
        list(self.onto.triples((None, self.crowracle.CROW.hasId, None)))
        print(time.time() - start)

        start = time.time()
        self.get_logger().info(str(self.crowracle.getTangibleObjects()))
        print(time.time() - start)


def main():
    rclpy.init()
    ot = OntoTester()
    ot.test_object()
    rclpy.spin(ot)


if __name__ == "__main__":
    main()
