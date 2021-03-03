import rdflib
from rdflib import URIRef, BNode, Literal
from rdflib.term import Identifier
from rdflib.namespace import Namespace, RDF, RDFS, OWL, FOAF, XSD
from collections import defaultdict
from collections.abc import Iterable
from itertools import chain
from rdflib.extras.infixowl import classOrIdentifier, Property
import yaml
from warnings import warn
import cv2
import os
import argparse
from owlrl import DeductiveClosure, OWLRL_Semantics
import re

# %%ArgParser
parser = argparse.ArgumentParser()
parser.add_argument("--onto_file", "-o", default="ontology/onto_draft_01_debug.owl")
args = parser.parse_args()

# %%Initialization
onto_file = args.onto_file
print(onto_file)
split_name_re = re.compile(r"([\w\/]+)\.?")
# split_name_re = re.compile(r".*\.(\w*)\.(\w*)")

# %%Load onto
ONTO_IRI = "http://www.semanticweb.org/crow/ontologies/2019/6/onto_draft_01"
CROW = Namespace(f"{ONTO_IRI}#")

onto = rdflib.Graph()
onto.load(onto_file)
onto.bind("crow", CROW)

# %%Functions
def getBaseNameAndNS(obj_name, build_name=None):
    if "." in obj_name or build_name is None:
        # if obj_name.count(".") > 1:
        *nspaces, obj_name = split_name_re.findall(obj_name)
    else:
        nspaces = []
        
    NS = Namespace(URIRef(ONTO_IRI) + ("/" + build_name if build_name is not None else "") + (("/" + "/".join(nspaces)) if len(nspaces) > 0 else "") + "#")
    return obj_name, NS

# %%Add detected object
def addDetectedObject(object_name, location):
    # find template object
    prop_range = list(onto.objects(CROW.hasDetectorName, RDFS.range))[0]
    template = list(onto.subjects(CROW.hasDetectorName, Literal(object_name, datatype=prop_range)))[0]
    print(str(template))
    all_props = list(onto.triples((template, None, None)))
    template_type = list(onto.objects(template, RDF.type))[0]
    num_subjects = len(list(onto.subjects(RDF.type, template_type)))
    individual_name = object_name + '_' + str(num_subjects)

    # Add common object properties
    for prop in all_props:
        print(str(prop)+'\n')
        onto.add((CROW[individual_name], prop[1], prop[2]))
        #add idividual_name_ns#hole1 for all objectParts

    # Add AbsoluteLocaton (object specific)
    PART = Namespace(f"{ONTO_IRI}/{individual_name}#") #ns for each object (/cube_holes_1#)
    prop_name = PART['xyzAbsoluteLocation']
    prop_range = list(onto.objects(CROW.hasAbsoluteLocation, RDFS.range))[0]
    onto.add((prop_name, RDF.type, prop_range))
    onto.add((prop_name, CROW.x, Literal(location[0], datatype=XSD.float)))
    onto.add((prop_name, CROW.y, Literal(location[1], datatype=XSD.float)))
    onto.add((prop_name, CROW.z, Literal(location[2], datatype=XSD.float)))
    onto.set((CROW[individual_name], CROW.hasAbsoluteLocation, prop_name))

    if len(individual_names) == 0:
        T_ref2world = []
        #first object defines reference coordinate frame
        #rel_loc = old one 
    else:
        rel_loc = []

    return {'name': individual_name, 'template': template}

def addAssembledObject(object_name, location):
    PART = Namespace(f"{ONTO_IRI}/{object_name['name']}#") #ns for each object (/cube_holes_1#)
    prop_name = PART['xyzAbsoluteLocation']
    onto.set((prop_name, CROW.x, Literal(location[0], datatype=XSD.float)))
    onto.set((prop_name, CROW.y, Literal(location[1], datatype=XSD.float)))
    onto.set((prop_name, CROW.z, Literal(location[2], datatype=XSD.float)))
    #if enables/disables in connection:
    changeEnabled(object_name, 'thread2', False) #acc to connection

def changeEnabled(object_name, part_name, value):
    template_type = list(onto.objects(object_name['template'], RDF.type))[0]
    template_name, _ = getBaseNameAndNS(template_type)
    PART = Namespace(f"{ONTO_IRI}/{template_name}#") #ns for each object (/cube_holes_1#)
    part_name = PART[part_name]
    prop_name = PART['enabled']
    prop_range = list(onto.objects(CROW.isEnabled, RDFS.range))[0]
    onto.add((prop_name, RDF.type, prop_range))
    onto.add((prop_name, CROW.isBool, Literal(value, datatype=XSD.boolean)))
    onto.set((CROW[part_name], CROW.isEnabled, prop_name))

# %% Add objects
individual_names = []
individual_names.append(addDetectedObject('cube_holes', [2.0, 2.0, 3.0]))
individual_names.append(addDetectedObject('peg_screw', [2.0, 2.0, 3.0]))

addAssembledObject(individual_names[1], [1.0, 0.0, 0.0])

# %% Save
outonto_file = "ontology/onto_draft_01_debug.owl"

onto.serialize(outonto_file)
