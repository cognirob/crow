import rclpy
from rclpy.node import Node
import curses
from curses.textpad import Textbox, rectangle
from crow_ontology.crowracle_client import CrowtologyClient
from rdflib.namespace import Namespace, RDF, RDFS, OWL, FOAF
from rdflib import URIRef, BNode, Literal, Graph
from rdflib.term import Identifier
import time

CROW = Namespace("http://imitrob.ciirc.cvut.cz/ontologies/crow#")


class OntoTester(Node):
    GUI_UPDATE_INTERVAL = 0.3

    def __init__(self, node_name="onto_tester"):
        super().__init__(node_name)
        self.crowracle = CrowtologyClient(node=self)
        self.onto = self.crowracle.onto
        # self.get_logger().info(self.onto)
        self.create_timer(self.GUI_UPDATE_INTERVAL, self.update_cb)

    def update_cb(self):
        self.stdscr.clear()
        self.stdscr.addstr(0, 0, "= Entities in ontology =")
        # self.get_logger().info(str(list(self.crowracle.getTangibleObjectClasses())))
        for i, (s) in enumerate(self.crowracle.getTangibleObjects()):
            self.stdscr.addstr(1 + i, 0, f"{s}")
        self.stdscr.refresh()

    def start(self):
        self.get_logger().info("Initialize!!!")
        self.stdscr = curses.initscr()
        curses.noecho()
        self.stdscr.addstr(0, 0, "= Entities in ontology =")
        self.stdscr.refresh()
        # editwin = curses.newwin(5,30, 2,1)

        # rectangle(stdscr, 1,0, 1+5+1, 1+30+1)
        # stdscr.refresh()

        # box = Textbox(editwin)

        # # Let the user edit until Ctrl-G is struck.
        # box.edit()

        # # Get resulting contents
        # message = box.gather()

    def destroy_node(self):
        curses.echo()
        curses.endwin()
        super().destroy_node()

def main():
    rclpy.init()
    ot = OntoTester()
    ot.start()
    rclpy.spin(ot)
    ot.destroy_node()


if __name__ == "__main__":
    main()
