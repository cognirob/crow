import rclpy
from rclpy.node import Node
import curses
import time
from curses.textpad import Textbox, rectangle
from crow_ontology.crowracle import Crowtology
from rdflib.namespace import Namespace, RDF, RDFS, OWL, FOAF, XSD
from rdflib import URIRef, BNode, Literal, Graph
from rdflib.term import Identifier

ONTO_IRI = "http://imitrob.ciirc.cvut.cz/ontologies/crow"
CROW = Namespace(f"{ONTO_IRI}#")


class OntoAdder(Node):

    def __init__(self, node_name="onto_adder"):
        super().__init__(node_name)
        self.crowracle = Crowtology(True)
        self.onto = self.crowracle.onto
        # self.get_logger().info(self.onto)

    def add_detected_object(self, object_name, location):
        self.get_logger().info("Adding detected object {} at location {}".format(object_name, location))
        # Find template object
        prop_range = list(self.onto.objects(subject=CROW.hasDetectorName, predicate=RDFS.range))[0]        
        template = list(self.onto.subjects(CROW.hasDetectorName, Literal(object_name, datatype=prop_range)))[0]
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

        # if len(individual_names) == 0:
        #     T_ref2world = []
        #     #first object defines reference coordinate frame
        #     #rel_loc = old one 
        # else:
        #     rel_loc = []

        return {'name': individual_name, 'template': template}

    def add_assembled_object(self, object_name, location):
        self.get_logger().info("Setting properties of assembled object {} at location {}".format(object_name, location))
        PART = Namespace(f"{ONTO_IRI}/{object_name['name']}#") #ns for each object (/cube_holes_1#)
        prop_name = PART.xyzAbsoluteLocation
        self.onto.set((prop_name, CROW.x, Literal(location[0], datatype=XSD.float)))
        self.onto.set((prop_name, CROW.y, Literal(location[1], datatype=XSD.float)))
        self.onto.set((prop_name, CROW.z, Literal(location[2], datatype=XSD.float)))
        #if enables/disables in connection:
        self.change_enabled(object_name, 'thread2', False) #acc to connection

    def change_enabled(self, object_name, part_name, value):
        PART = Namespace(f"{ONTO_IRI}/{object_name['name']}/{part_name}#") #ns for each object (/cube_holes_1#)
        OBJ = Namespace(f"{ONTO_IRI}/{object_name['name']}#") #ns for each object (/cube_holes_1#)
        part_name = OBJ[part_name]
        prop_name = PART.enabled
        prop_range = list(self.onto.objects(CROW.isEnabled, RDFS.range))[0]
        self.onto.add((prop_name, RDF.type, prop_range))
        self.onto.add((prop_name, CROW.isBool, Literal(value, datatype=XSD.boolean)))
        self.onto.set((part_name, CROW.isEnabled, prop_name))

def main():
    rclpy.init()
    time.sleep(10)
    adder = OntoAdder()
    individual_names = []
    # for demo: add three objectes and assemble Peg
    individual_names.append(adder.add_detected_object('cube_holes', [2.0, 2.0, 3.0]))
    individual_names.append(adder.add_detected_object('cube_holes', [4.0, 4.0, 3.0]))
    individual_names.append(adder.add_detected_object('peg_screw', [2.0, 2.0, 3.0]))

    adder.add_assembled_object(individual_names[2], [1.0, 0.0, 0.0])

    rclpy.spin(adder)


if __name__ == "__main__":
    main()