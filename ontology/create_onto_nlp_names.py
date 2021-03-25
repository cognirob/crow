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
parser.add_argument("--onto_file", "-o", default="./src/crow/ontology/onto_draft_02.owl")
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

onto = rdflib.Graph()
onto.load(onto_file)
onto.bind("crow", CROW)

def add_names_class(names_defs, language, property):
    for entity, names in names_defs['classes'][language].items():
        onto_name = list(onto.triples((CROW[entity], RDF.type, OWL.Class)))
        if onto_name == []:
            print('not a valid CROW class {}'.format(entity))
        for onto_name_class in onto_name:
            for name in names:
                onto.add((onto_name_class[0], property, Literal(name, datatype=XSD.string)))

                # # Check if added
                # class_with_name = list(onto.subjects(CROW.hasNlpName, Literal(name, datatype=XSD.string)))
                # print('wished class', onto_name_class[0])
                # print('found class', class_with_name)
def add_names_color(names_defs, language, property):
    for entity, names in names_defs['colors'][language].items():
        onto_name = list(onto.triples((CROW[entity], RDF.type, CROW.NamedColor)))
        if onto_name == []:
            print('not a valid CROW individual {}'.format(entity))
        for onto_name_class in onto_name:
            for name in names:
                onto.add((onto_name_class[0], property, Literal(name, datatype=XSD.string)))

                # # Check if added
                # class_with_name = list(onto.subjects(CROW.hasNlpName, Literal(name, datatype=XSD.string)))
                # print('wished class', onto_name_class[0])
                # print('found class', class_with_name)

def buildGraph(names_file, onto, recipe_name=None):
    # %%Load YAML
    with open(names_file, "r") as f:
        names_defs = yaml.safe_load(f)
    print(f"Parsing {names_file}\n")


    # Add names to classes
    if 'classes' in names_defs:
        add_names_class(names_defs, 'en', CROW.hasNlpNameEN)
        add_names_class(names_defs, 'cz', CROW.hasNlpNameCZ)
    
    # Add names to colors
    if 'colors' in names_defs:
        add_names_color(names_defs, 'en', CROW.hasNlpNameEN)
        add_names_color(names_defs, 'cz', CROW.hasNlpNameCZ)

# %% Do
buildGraph(names_file, onto)
# %% Draw
outonto_file = "./src/crow/ontology/onto_draft_03.owl"
#utonto_file = onto_file

onto.serialize(outonto_file)
