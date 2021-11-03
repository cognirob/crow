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
    MAX_LINES = 14
    COLUMN_WIDTH = 25
    FIX_MINIMUM_SIZE_WHEN_CREATED = False

    def create(self):
        self.min_l = 15
        self.node = self.parentApp.node
        self.crowracle = self.parentApp.crowracle

        # setup vis
        self.objects = {}
        self.elements = []
        self.empty_lines = []
        self.offsets = [0, 22, 15, 24, 14, 25, 40]
        self.fields = ["uri", "cls", "loc", "enabled", "area", "uuid", "tracked"]

        for i, (f, o) in enumerate(zip(self.fields, self.offsets)):
            if i > 0:
                self.nextrelx += o
                self.nextrely -= 1
            self.add(npyscreen.Textfield, name=f"{f}", value=f, editable=False)
        self.nextrelx -= sum(self.offsets)

        for n in range(self.MAX_LINES):
            element = {}
            for i, (f, o) in enumerate(zip(self.fields, self.offsets)):
                if i > 0:
                    self.nextrelx += o
                    self.nextrely -= 1
                elm = self.add(npyscreen.Textfield, name=f"{f}_{n}", editable=False)
                element[f] = elm

            self.elements.append(element)
            self.empty_lines.append(True)
            self.nextrelx -= sum(self.offsets)

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

    def _shorten_uri(self, input_uri: str) -> str:
        """Replaces the namespace part of the URI with a short namespace name.

        Args:
            input_uri (str): The URI to shorten.

        Returns:
            str: Shortened version of the URI.
        """
        input_uri = input_uri.replace(CROW, "")
        return input_uri

    def update(self):
        objs = self.crowracle.get_object_visualization()
        updated_objs = []
        current_objects = self.objects.keys()
        new = None
        for obj, cls, uuid, id, did, x, y, z, area, tracked in objs:
            obj = self._shorten_uri(obj)
            new = obj not in current_objects
            if new:  # new object
                empty_line = np.where(self.empty_lines)[0]
                if len(empty_line) > 0:
                    empty_line = empty_line[0]
                else:
                    continue  # TODO: something safer

                self.empty_lines[empty_line] = False
                entries = self.elements[empty_line]
                entries["uri"].value = str(obj)
                self.objects[obj] = [entries, empty_line, True, True]
            else:  # already detected object
                entries, line, alive, new = self.objects[obj]
                if obj in updated_objs:  # object was already processed but some part of the query returned multiple values
                    if area is None:
                        # print(obj, cls, uuid, id, did, x, y, z, area)
                        continue
                    if entries["area"].value != area:
                        entries["area"].value += f", {area}"
                    continue

            updated_objs.append(obj)
            entries["cls"].value = self._shorten_uri(cls)
            entries["loc"].value = f"[{x.toPython():0.3f}, {y.toPython():0.3f}, {z.toPython():0.3f}]"
            if id is None:
                if did is None:
                    entries["enabled"].value = ""
                else:
                    entries["enabled"].value = "disabled"
                    entries["enabled"].color = 'WARNING'
            else:
                entries["enabled"].value = "enabled"
                entries["enabled"].color = 'SAFE'
            entries["area"].value = area or ""
            entries["uuid"].value = uuid
            entries["tracked"].value = str(tracked) if tracked is not None else ""

        tobedeleted = []
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
                    entries["enabled"].color = 'DEFAULT'
                    for f in self.fields:
                        entries[f].value = ""
                    tobedeleted.append(obj)

        for obj in tobedeleted:
            del self.objects[obj]

        self.display()

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
