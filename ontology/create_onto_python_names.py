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
parser.add_argument("--onto_file", "-o", default="./crow/crow_ontology/data/onto_draft_03.owl")
args = parser.parse_args()

# %%Initialization
#names_file = args.names_file
names_file = "./crow/ontology/assembly/properties_python_names.yaml"
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


def buildGraph(names_file, onto, recipe_name=None):
    # %%Load YAML
    with open(names_file, "r") as f:
        names_defs = yaml.safe_load(f)
    print(f"Parsing {names_file}\n")

    # Add python names to properties
    if 'data_properties' in names_defs:
        for k, v in iter(names_defs['data_properties'].items()):
            kns = k.rsplit('#')[0]
            kNS = Namespace(f"{kns}#")
            sub = kNS[k.rsplit('#')[-1]]
            if len(list(onto.objects(sub, python_name))) < 1:
                onto.add((sub, python_name, Literal(v, datatype=XSD.string)))
    if 'obj_properties' in names_defs:
        for k, v in iter(names_defs['obj_properties'].items()):
            kns = k.rsplit('#')[0]
            kNS = Namespace(f"{kns}#")
            sub = kNS[k.rsplit('#')[-1]]
            if len(list(onto.objects(sub, python_name))) < 1:
                onto.add((sub, python_name, Literal(v, datatype=XSD.string)))

    # Add world names to objects
    if 'objects' in names_defs:
        for k, v in iter(names_defs['objects'].items()):
            kns = k.rsplit('#')[0]
            kNS = Namespace(f"{kns}#")
            sub = kNS[k.rsplit('#')[-1]]
            if len(list(onto.objects(sub, CROW.world_name))) < 1:
                onto.add((sub, CROW.world_name, Literal(v, datatype=XSD.string)))


# %% Do
buildGraph(names_file, onto)
# %% Draw
outonto_file = "./crow/crow_ontology/data/onto_draft_03.owl"
#utonto_file = onto_file

onto.serialize(outonto_file)
