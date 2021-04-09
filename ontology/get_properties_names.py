import rdflib
from rdflib import URIRef, BNode, Literal
from rdflib.term import Identifier
from rdflib.namespace import Namespace, RDF, RDFS, OWL, FOAF, XSD
import yaml
import os
import argparse
import re

# %%ArgParser
parser = argparse.ArgumentParser()
#parser.add_argument("names_file")
parser.add_argument("--onto_file", "-o", default="./src/crow/crow_ontology/data/onto_draft_03.owl")
args = parser.parse_args()

# %%Initialization
#names_file = args.names_file
names_file = "./src/crow/ontology/assembly/nlp_names.yaml"
onto_file = args.onto_file
print(onto_file)
split_name_re = re.compile(r"([\w\/]+)\.?")
# split_name_re = re.compile(r".*\.(\w*)\.(\w*)")

# %%Load onto
ONTO_IRI = "http://imitrob.ciirc.cvut.cz/ontologies/crow"
CROW = Namespace(f"{ONTO_IRI}#")
OWL_READY = "http://www.lesfleursdunormal.fr/static/_downloads/owlready_ontology.owl"
python_name = Namespace(f"{OWL_READY}#").python_name

onto = rdflib.Graph()
onto.load(onto_file)
onto.bind("crow", CROW)

all_properties = {'obj_properties': {}, 'data_properties': {}}
obj_properties = list(onto.subjects(RDF.type, OWL.ObjectProperty))
for x in obj_properties:
    name = list(onto.objects(x, python_name))
    if x == CROW.hasColor:
        print(name)
    if len(name) > 0:
        all_properties['obj_properties'][x.toPython()] = name[0].toPython()
    else:
        all_properties['obj_properties'][x.toPython()] = None

data_properties = list(onto.subjects(RDF.type, OWL.DatatypeProperty))
for x in data_properties:
    name = list(onto.objects(x, python_name))
    if x == CROW.hasColor:
        print(name)
    if len(name) > 0:
        all_properties['data_properties'][x.toPython()] = name[0].toPython()
    else:
        all_properties['data_properties'][x.toPython()] = None

with open('properties_python_names.yaml', 'w') as f:
    yaml.dump(all_properties, f)
