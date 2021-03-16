import rclpy
from rclpy.node import Node
import curses
import time
import numpy as np
from curses.textpad import Textbox, rectangle
from crow_ontology.crowracle_client import CrowtologyClient
from rdflib.namespace import Namespace, RDF, RDFS, OWL, FOAF, XSD
from rdflib import URIRef, BNode, Literal, Graph
from rdflib.term import Identifier

ONTO_IRI = "http://imitrob.ciirc.cvut.cz/ontologies/crow"
CROW = Namespace(f"{ONTO_IRI}#")

def distance(entry):
    return entry[-1]


class OntoAdder(Node):

    def __init__(self, node_name="onto_adder"):
        super().__init__(node_name)
        self.crowracle = CrowtologyClient(node=self)
        self.onto = self.crowracle.onto
        # self.get_logger().info(self.onto)
        self.loc_threshold = 0.05 # object detected within 5cm from an older detection will be considered as the same one
        self.id = self.get_last_id() + 1
        self.get_logger().info("There are already {} detected objects in the database.".format(self.id - 1))

    def get_last_id(self):
        all_detected = list(self.onto.objects(None, CROW.hasId))
        all_detected = [int(id.split('od_')[-1]) for id in all_detected]
        if len(all_detected) > 0:
            return max(all_detected)
        else:
            return -1

    def process_detected_object(self, object_name, location, timestamp):
        self.get_logger().info("Processing detected object {} at location {}.".format(object_name, location))
        prop_range = list(self.onto.objects(subject=CROW.hasDetectorName, predicate=RDFS.range))[0]
        corresponding_objects = list(self.onto.subjects(CROW.hasDetectorName, Literal(object_name, datatype=prop_range)))
        already_detected = []
        for x in corresponding_objects:
            if len(list(self.onto.objects(subject=x, predicate=CROW.hasTimestamp))) > 0:
                old_loc_obj = list(self.onto.objects(x, CROW.hasAbsoluteLocation))[0]
                old_loc = [float(list(self.onto.objects(old_loc_obj, x))[0]) for x in [CROW.x, CROW.y, CROW.z]]
                dist = (np.linalg.norm(np.asarray(old_loc) - np.asarray(location)))
                already_detected.append([x, dist])
        if len(already_detected) > 0:
            already_detected.sort(key=distance)
            closest = already_detected[0]
            if closest[-1] <= self.loc_threshold: # update timestamp and loc of matched already detected object
                individual_name = closest[0].split('#')[-1]
                self.get_logger().info("Object {} already detected, updating timestamp to {} and location to {}.".format(individual_name, timestamp, location))
                self.onto.set((closest[0], CROW.hasTimestamp, Literal(timestamp, datatype=XSD.dateTimeStamp)))
                abs_loc = list(self.onto.objects(CROW[individual_name], CROW.hasAbsoluteLocation))[0]
                self.onto.set((abs_loc, CROW.x, Literal(location[0], datatype=XSD.float)))
                self.onto.set((abs_loc, CROW.y, Literal(location[1], datatype=XSD.float)))
                self.onto.set((abs_loc, CROW.z, Literal(location[2], datatype=XSD.float)))
            else: # add new detected object
                individual_name = self.add_detected_object(object_name, location, timestamp, corresponding_objects[0])
        else: # add new detected object
                individual_name = self.add_detected_object(object_name, location, timestamp, corresponding_objects[0])
        return individual_name

    def add_detected_object(self, object_name, location, timestamp, template):
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

    def add_assembled_object(self, object_name, location):
        self.get_logger().info("Setting properties of assembled object {} at location {}.".format(object_name, location))
        PART = Namespace(f"{ONTO_IRI}/{object_name}#") #ns for each object (/cube_holes_1#)
        prop_name = PART.xyzAbsoluteLocation
        self.onto.set((prop_name, CROW.x, Literal(location[0], datatype=XSD.float)))
        self.onto.set((prop_name, CROW.y, Literal(location[1], datatype=XSD.float)))
        self.onto.set((prop_name, CROW.z, Literal(location[2], datatype=XSD.float)))
        #if enables/disables in connection:
        self.change_enabled(object_name, 'thread2', False) #acc to connection

    def change_enabled(self, object_name, part_name, value):
        PART = Namespace(f"{ONTO_IRI}/{object_name}/{part_name}#") #ns for each object (/cube_holes_1#)
        OBJ = Namespace(f"{ONTO_IRI}/{object_name}#") #ns for each object (/cube_holes_1#)
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
    # for demo: add three objects and update location of the last one
    individual_names.append(adder.process_detected_object('cube_holes', [2.0, 2.0, 3.0], '2020-03-11T15:50:00Z'))
    individual_names.append(adder.process_detected_object('cube_holes', [4.0, 4.0, 3.0], '2020-03-11T15:55:00Z'))
    individual_names.append(adder.process_detected_object('peg_screw', [2.0, 2.0, 3.0], '2020-03-11T15:59:00Z'))
    individual_names.append(adder.process_detected_object('peg_screw', [2.001, 2.0, 3.0], '2020-03-11T15:59:00Z'))

    #adder.add_assembled_object(individual_names[2], [1.0, 0.0, 0.0]) #later change to (id, loc)

    rclpy.spin(adder)


if __name__ == "__main__":
    main()