import rclpy
from rclpy.node import Node
import curses
from curses.textpad import Textbox, rectangle
from crow_ontology.crowracle_client import CrowtologyClient
from rdflib.namespace import Namespace, RDF, RDFS, OWL, FOAF
from rdflib import URIRef, BNode, Literal, Graph
from rdflib.term import Identifier
from crow_vision_ros2.utils.test_point_in_polyhedron import test_in_hull
import time


class OntoTester(Node):

    def __init__(self, node_name="onto_tester"):
        super().__init__(node_name)
        self.crowracle = CrowtologyClient(node=self)
        self.onto = self.crowracle.onto
        # self.get_logger().info(self.onto)

    def test_object(self):
        self.get_logger().info("Found these classes of tangible objects in the database")
        start = time.time()
        qres = self.crowracle.getTangibleObjectClasses()
        for c in qres:
            self.get_logger().info(f"{c}")
        print(time.time() - start)

        self.get_logger().info("Found these objects in the scene (onto.triples)")
        start = time.time()
        res = list(self.onto.triples((None, self.crowracle.CROW.hasId, None)))
        self.get_logger().info(f"{res}")
        print(time.time() - start)

        self.get_logger().info("Found these tangible objects in the scene (getTangibleObjects)")
        start = time.time()
        self.get_logger().info(str(self.crowracle.getTangibleObjects()))
        print(time.time() - start)

        print('get_uri_from_nlp')
        uris = self.crowracle.get_uri_from_nlp('blue')
        print(str(uris))
        uris = self.crowracle.get_uri_from_nlp('cube')
        print(str(uris))

        print('get_nlp_from_uri')
        names = self.crowracle.get_nlp_from_uri(self.crowracle.CROW.COLOR_BLUE)
        print(names)
        names = self.crowracle.get_nlp_from_uri(self.crowracle.CROW.CUBE)
        print(names)
        names = self.crowracle.get_nlp_from_uri(self.crowracle.CROW.Peg)
        print(names)
        names = self.crowracle.get_nlp_from_uri(self.crowracle.CROW.Human)
        print(names)

        print('get_all_tangible_nlp')
        names = self.crowracle.get_all_tangible_nlp(language='EN')
        print(names)

        print('get_color_of_obj')
        uri = self.crowracle.get_color_of_obj(self.crowracle.CROW.CUBE)
        print(uri)

        print('get_color_of_obj_nlp')
        names = self.crowracle.get_color_of_obj_nlp('cube')
        print(names)

        print('get_obj_of_color')
        uris = self.crowracle.get_obj_of_color(self.crowracle.CROW.COLOR_GREEN)
        print(uris)

        print('get_obj_of_color_nlp')
        names = self.crowracle.get_obj_of_color_nlp('gold', all=False)
        print(names)

        print('get_location_of_obj')
        uris = self.crowracle.get_location_of_obj(self.crowracle.CROW.CUBE)
        print(uris)

        print('find_obj_nlp')
        locations = self.crowracle.find_obj_nlp('sphere', all=False)
        print(locations)

        print('find_obj_of_color_nlp')
        locations = self.crowracle.find_obj_of_color_nlp('gold', all=False)
        print(locations)

        print('get_colors')
        colors = self.crowracle.getColors()
        print(colors)

        print('get_colors_nlp')
        colors = self.crowracle.get_colors_nlp()
        print(colors)

        print('get_pcl_dimensions_of_obj')
        dims = self.crowracle.get_pcl_dimensions_of_obj(self.crowracle.CROW.cube_holes_od_2)
        print(dims)

        print('get_fixed_dimensions_of_obj')
        dims = self.crowracle.get_fixed_dimensions_of_obj(self.crowracle.CROW.cube_holes_od_2)
        print(dims)
        
        q1 = {"color": self.crowracle.CROW.COLOR_LIGHT_BLUE}
        q2 = {"color": None}
        obj_cls1 = self.crowracle.CROW.Nut
        obj_cls2 = None
        print('get_obj_of_properties')
        uris = self.crowracle.get_obj_of_properties(obj_cls1, q1, all=True)
        print(uris)
        print('get_obj_of_properties')
        uris = self.crowracle.get_obj_of_properties(obj_cls2, q2, all=True)
        print(uris)

        print('get_obj_of_id')
        objs = self.crowracle.get_obj_of_id('od_0')
        print(objs)

        print('get_name_from_prop')
        name = self.crowracle.get_name_from_prop(self.crowracle.CROW.hasColor)
        print(name)

        print('get_prop_from_name')
        uri = self.crowracle.get_prop_from_name('color')
        print(uri)

        print('get_all_tools')
        uri = self.crowracle.get_all_tools(all=True)
        print(uri)

        print('get_all_tools')
        uri = self.crowracle.get_all_tools(all=False)
        print(uri)

        print('get_tang_with_props')
        list_of_dicts = self.crowracle.getTangibleObjectsProps()
        print(list_of_dicts)

        print('get_filter_object_properties')
        dict_of_dicts = self.crowracle.get_filter_object_properties()
        print(dict_of_dicts)

        print('get_storage')
        q1 = {"name": 'storage1'}
        uris = self.crowracle.get_obj_of_properties(self.crowracle.CROW.StorageSpace, q1, all=True)
        print(uris)

        print('test in hull')
        poly = self.crowracle.get_polyhedron(uris)
        res = test_in_hull([1., 0.5, 2.5], poly)
        print(res)

        print('get_actions')
        q1 = {"name": 'Holding something'}
        uris = self.crowracle.get_obj_of_properties(self.crowracle.CROW.Action, q1, all=True)
        print(uris)

        print('done')

def main():
    rclpy.init()
    ot = OntoTester()
    ot.test_object()
    rclpy.spin(ot)


if __name__ == "__main__":
    main()
