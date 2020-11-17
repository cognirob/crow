# -*- coding: utf-8 -*-
"""
Created on Fri Nov 13 20:59:47 2020

@author: rados
"""
import rdflib
from rdflib import URIRef, BNode, Literal
from rdflib.term import Identifier
from rdflib.namespace import Namespace, RDF, RDFS, OWL, FOAF
from collections import defaultdict
from collections.abc import Iterable
from itertools import chain
from rdflib.extras.infixowl import classOrIdentifier
import pygraphviz as pgv
import yaml
from warnings import warn
import cv2


# %%Load onto
CROW = Namespace("http://www.semanticweb.org/crow/ontologies/2019/6/onto_draft_01#")

onto = rdflib.Graph()
onto.load("onto_draft_01.owl")
onto.bind("crow", CROW)

# %%Load YAML
with open("build_dog.yaml", "r") as f:
    recipe = yaml.safe_load(f)

# %%Build graph
graph = pgv.AGraph(strict=False)
graph.node_attr['style']='filled'

graph.add_node(recipe["assembly_name"], shape="note", fillcolor="white")

for entity, props in recipe["objects"].items():
    try:
        next(onto.triples((CROW[props["type"]], RDF.type, OWL.Class)))
    except StopIteration as e:
        warn(f"Entity {entity} not found in Ontology!")
    else:
        superClasses = list(onto.transitive_objects(CROW[props["type"]], RDFS.subClassOf))
        if CROW.WorkMaterial in superClasses:
            if CROW.Workpiece in superClasses:
                graph.add_node(entity, label=entity + f"\n{props['type']}", shape="box", fillcolor="azure2")
            elif CROW.Consumable in superClasses:
                graph.add_node(entity, label=entity + f"\n{props['type']}", shape="box", fillcolor="gray")

for entity, props in recipe["connections"].items():
    try:
        next(onto.triples((CROW[props["type"]], RDF.type, OWL.Class)))
    except StopIteration as e:
        warn(f"Entity {entity} not found in Ontology!")
    else:
        superClasses = list(onto.transitive_objects(CROW[props["type"]], RDFS.subClassOf))
        label = entity + f"\n{props['type'][:props['type'].index('Connection')]}"
        graph.add_node(entity, label=label, shape="oval", fillcolor="lavender")
        if CROW.AssemblyBinaryConnection in superClasses:
            if CROW.InsertConnection in superClasses:
                a = props["shaft"]
                b = props["hole"]
            else:
                a = props["provider"]
                b = props["consumer"]
            graph.add_edge(a, entity)
            graph.add_edge(entity, b)
            
for i, props in enumerate(recipe["relations"]):
    try:
        next(onto.triples((CROW[props["type"]], RDF.type, OWL.Class)))
    except StopIteration as e:
        warn(f"Entity {props['type']} not found in Ontology!")
    else:
        name = str(next(onto.triples((CROW[props["type"]], CROW.representation, None)))[2])
        # name = props["type"][0]
        # name = props["type"][:props["type"].index("Relation")]
        uname = name + str(i)
        superClasses = list(onto.transitive_objects(CROW[props["type"]], RDFS.subClassOf))
        graph.add_node(uname, label=name, shape="circle", color="red", fillcolor="indianred1")
        if CROW.UnorderedRelation in superClasses:
            connections = props["connections"]
            for conn in connections:
                graph.add_edge(conn, uname, color="red")
        if CROW.RelationWithReference in superClasses:
            graph.add_edge(props["reference"], uname, color="red", style="dashed")
            
for i, props in enumerate(recipe["order_hints"]):
    try:
        next(onto.triples((CROW[props["type"]], RDF.type, OWL.Class)))
    except StopIteration as e:
        warn(f"Entity {props['type']} not found in Ontology!")
    else:
        name = str(next(onto.triples((CROW[props["type"]], CROW.representation, None)))[2])
        uname = name + str(i)
        superClasses = list(onto.transitive_objects(CROW[props["type"]], RDFS.subClassOf))
        graph.add_node(uname, label=name, shape="square", color="darkgreen", fillcolor="honeydew")
        if CROW.SequentialOrder in superClasses:
            graph.add_edge(props["first"], uname, color="darkgreen", fillcolor="honeydew", arrowHead="vee", dir="forward")
            graph.add_edge(uname, props["then"], color="darkgreen", fillcolor="honeydew", arrowHead="vee", dir="forward")
        
graph.layout(prog='dot')
file = "file.png"
graph.draw(file, prog="dot")
image = cv2.imread("file.png")
cv2.imshow(file, image)
cv2.waitKey(0)
cv2.destroyAllWindows()