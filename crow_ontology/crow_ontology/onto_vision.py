import rclpy
from rclpy.node import Node
import curses
from curses.textpad import Textbox, rectangle
from crow_ontology.crowracle_client import CrowtologyClient
from rdflib.namespace import Namespace, RDF, RDFS, OWL, FOAF
from rdflib import URIRef, BNode, Literal, Graph
from rdflib.term import Identifier
import time
from npyscreen.wgmultiline import MultiLine
import npyscreen
import os
from threading import Thread
import numpy as np


CROW = Namespace("http://imitrob.ciirc.cvut.cz/ontologies/crow#")


class MainForm(npyscreen.TitleForm):
    MAX_LINES = 15
    COLUMN_WIDTH = 20

    def create(self):
        self.min_l = 15
        self.node = self.parentApp.node
        self.crowracle = self.parentApp.crowracle

        # setup vis
        self.objects = {}
        self.elements = []
        self.empty_lines = []
        for n in range(self.MAX_LINES):
            uri = self.add(npyscreen.Textfield, editable=False)
            self.nextrelx += self.COLUMN_WIDTH
            self.nextrely -= 1
            loc = self.add(npyscreen.Textfield, editable=False)
            self.nextrelx += self.COLUMN_WIDTH
            self.nextrely -= 1
            area = self.add(npyscreen.Textfield, editable=False)
            self.nextrelx += self.COLUMN_WIDTH
            self.nextrely -= 1
            uuid = self.add(npyscreen.Textfield, editable=False)
            self.nextrelx += self.COLUMN_WIDTH
            self.nextrely -= 1
            self.elements.append((uri, loc, area, uuid))
            self.empty_lines.append(True)
            self.nextrelx -= self.COLUMN_WIDTH * 4

        # start the updates
        self.th = Thread(target=self.spin, daemon=True)
        self.th.start()
        self.node.create_timer(0.01, self.update)

    def spin(self):
        rclpy.spin(self.node)

    def beforeEditing(self):
        pass

    def afterEditing(self):
        self.parentApp.switchFormPrevious()

    def update(self):
        objs = self.crowracle.get_object_visualization()
        updated_objs = []
        current_objects = self.objects.keys()
        new = None
        for obj, uuid, id, did, x, y, z, area in objs:
            new = obj not in current_objects
            updated_objs.append(obj)
            if new:  # new object
                empty_line = np.where(self.empty_lines)[0]
                if len(empty_line) > 0:
                    empty_line = empty_line[0]
                else:
                    continue  # TODO: something safer

                self.empty_lines[empty_line] = False
                line = self.elements[empty_line]
                entries = {
                    "uri": line[0],
                    "loc": line[1],
                    "area": line[2],
                    "uuid": line[3],
                }
                entries["uri"] = obj
                self.objects[obj] = entries, empty_line, True, True
            else:  # already detected object
                entries, line, alive, new = self.objects[obj]

            entries["loc"] = f"[{x:0.3f}, {y:0.3f}, {z:0.3f}]"
            entries["area"] = area
            entries["uuid"] = uuid

        for obj, (entries, line, alive, new) in self.objects.items():
            if new:
                self.objects[obj][3] = False
                entries["uri"].color = 'SAFE'
            else:
                if alive:
                    if obj in updated_objs:
                        entries["uri"].color = 'DEFAULT'
                    else:
                        self.objects[obj][2] = False
                        entries["uri"].color = 'DANGER'
                else:
                    self.empty_lines[line] = True
                    entries["uri"].color = 'DEFAULT'
                    entries["uri"] = ""
                    entries["loc"] = ""
                    entries["area"] = ""
                    entries["uuid"] = ""


class OViz(npyscreen.NPSAppManaged):

    def __init__(self, node):
        super().__init__()
        npyscreen.BufferPager.DEFAULT_MAXLEN = 500
        npyscreen.Popup.DEFAULT_COLUMNS = 100
        npyscreen.PopupWide.DEFAULT_LINES = 20
        npyscreen.PopupWide.SHOW_ATY = 1
        # initialize the ontology client
        self.node = node
        self.crowracle = CrowtologyClient(node=self.node)

        self.onto = self.crowracle.onto
        self.node.get_logger().set_level(rclpy.logging.LoggingSeverity.ERROR)

    def onStart(self):
        # npyscreen.setTheme(npyscreen.Themes.TransparentThemeLightText)
        self.addForm("MAIN", MainForm, name="Monitor")
        return super().onStart()

    def onCleanExit(self):
        print("Exited cleanly")
        return super().onCleanExit()


def main():
    os.environ['ESCDELAY'] = "0.1"
    rclpy.init()
    node = Node("system_monitor_node")
    ot = OViz(node)
    try:
        ot.run()
    except KeyboardInterrupt:
        ot.switchForm(None)
        print("User requested shutdown.")


if __name__ == '__main__':
    main()
