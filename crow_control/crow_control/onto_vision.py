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
        self.old_objects = {}

    def update_cb(self):
        # self.get_logger().info(str(list(self.crowracle.getTangibleObjectClasses())))
        new_objects = {}
        for s in self.crowracle.getTangibleObjects_nocls():
            uri = s
            try:
                loc = self.crowracle.get_location_of_obj(s)
                id = self.crowracle.get_id_of_obj(s)
            except Exception as e:
                loc = e
                id = "error"
            finally:
                new_objects[uri] = (loc, id)
        for l in self.crowracle.getStoragesProps():
            new_objects[l["uri"]] = ("storage", l["name"])
        for l in self.crowracle.getPositionsProps():
            new_objects[l["uri"]] = ("position", l["name"])
        self.render(new_objects)

    def render(self, new_objects):
        self.stdscr.clear()
        self.stdscr.addstr(0, 0, "= Entities in ontology =")
        combined_objects = {**self.old_objects, **new_objects}
        for i, (uri, (loc, id)) in enumerate(combined_objects.items()):
            dead = False
            if uri in self.old_objects:
                if uri not in new_objects:
                    dead = True
                    flags = curses.color_pair(3)
                else:
                    flags = curses.color_pair(0)
            elif uri in new_objects:
                flags = curses.color_pair(2)
            else:
                flags = curses.color_pair(5)
            if not dead and (id is None or "error" in id):
                flags = curses.color_pair(4)

            self.stdscr.addstr(1 + i, 0, f"{uri}", flags)
            if not dead:
                self.stdscr.addstr(1 + i, 100, f"loc: {loc}", flags)
                self.stdscr.addstr(1 + i, 200, f"ID: {id}", flags)

        self.stdscr.refresh()
        self.old_objects = new_objects

    def start(self):
        self.get_logger().info("Initialize!!!")
        self.stdscr = curses.initscr()
        curses.noecho()
        curses.start_color()
        curses.init_pair(1, curses.COLOR_BLUE, curses.COLOR_BLACK)
        curses.init_pair(2, curses.COLOR_GREEN, curses.COLOR_BLACK)
        curses.init_pair(3, curses.COLOR_RED, curses.COLOR_BLACK)
        curses.init_pair(4, curses.COLOR_YELLOW, curses.COLOR_BLACK)
        curses.init_pair(5, curses.COLOR_WHITE, curses.COLOR_RED)
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
        # print(self.crowracle.getStoragesProps())
        # print(self.crowracle.getPositionsProps())
        super().destroy_node()


def main():
    try:
        rclpy.init()
        ot = OntoTester()
        ot.start()
        rclpy.spin(ot)
    finally:
        ot.destroy_node()


if __name__ == "__main__":
    main()
