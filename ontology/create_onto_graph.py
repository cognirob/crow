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
import os
import argparse
from owlrl import DeductiveClosure, OWLRL_Semantics
import re


# %%ArgParser
parser = argparse.ArgumentParser()
parser.add_argument("build_name")
parser.add_argument("--onto_file", "-o", default="onto_draft_01.owl")
# args = parser.parse_args(["build_dog.yaml"])
# args = parser.parse_args(["sub_build_dog.yaml"])
args = parser.parse_args()

# %%Initialization
# build_name = "build_dog.yaml"
build_name = args.build_name
# onto_file = "onto_draft_01.owl"
# onto_file = "dog_build.owl"
onto_file = args.onto_file
split_name_re = re.compile(r"([\w\/]+)\.?")
# split_name_re = re.compile(r".*\.(\w*)\.(\w*)")

# %%Load onto
ONTO_IRI = "http://www.semanticweb.org/crow/ontologies/2019/6/onto_draft_01"
CROW = Namespace(f"{ONTO_IRI}#")

onto = rdflib.Graph()
onto.load("onto_draft_01.owl")
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

# def getUnqualifiedName

def getQualifiedName(obj_name, build_name=None):
    obj_name, BUILD = getBaseNameAndNS(obj_name, build_name)
    return BUILD[obj_name]

def checkIValidType(onto_type):
    try:
        next(onto.triples((onto_type, RDF.type, OWL.Class)))
    except StopIteration as e:
        return False
    else:
        return True

def checkConnectionObject(conn_name, obj_name, build_name):
    obj_name, BUILD = getBaseNameAndNS(obj_name, build_name)
    try:
        obj_onto_name, _, obj_type = next(onto.triples((BUILD[obj_name], RDF.type, None)))
    except StopIteration as e:
        warn(f"Object {obj_name} used in operation {conn_name} does not exist! Maybe you forgot to define it in 'objects' section?")
        return False
    else:
        superClasses = list(onto.transitive_objects(obj_type, RDFS.subClassOf))
        if not((CROW.WorkMaterial in superClasses) or (CROW.AssemblyGraph in superClasses)):
            warn(f"Object {obj_name} used in operation {conn_name} is not a sub-class of WorkMaterial or AssemblyGraph!")
            return False
        else:
            return True

def checkRelationOperation(rel_type, op_name, build_name):
    op_name, BUILD = getBaseNameAndNS(op_name, build_name)
    try:
        _, _, op_type = next(onto.triples((BUILD[op_name], RDF.type, None)))
    except StopIteration as e:
        warn(f"Operation {op_name} used in relation of type {rel_type} does not exist! Maybe you forgot to define it in 'operations' section?")
        return False
    else:
        superClasses = list(onto.transitive_objects(op_type, RDFS.subClassOf))
        if CROW.AssemblyOperation not in superClasses:
            warn(f"Operation {op_name} used in relation of type {rel_type} is not a subclass of AssemblyOperation!")
            return False
        else:
            return True

def checkOrderOperation(order_type, op_name, build_name):
    op_name, BUILD = getBaseNameAndNS(op_name, build_name)
    try:
        _, _, op_type = next(onto.triples((BUILD[op_name], RDF.type, None)))
    except StopIteration as e:
        warn(f"Operation {op_name} used in order hint of type {order_type} does not exist! Maybe you forgot to define it in 'operations' section?")
        return False
    else:
        superClasses = list(onto.transitive_objects(op_type, RDFS.subClassOf))
        if CROW.AssemblyOperation not in superClasses:
            warn(f"Operation {op_name} used in order hint of type {order_type} is not a subclass of AssemblyOperation!")
            return False
        else:
            return True

def checkReference(rel_type, ref_name, build_name):
    ref_name, BUILD = getBaseNameAndNS(ref_name, build_name)
    try:
        _, _, ref_type = next(onto.triples((BUILD[ref_name], RDF.type, None)))
    except StopIteration as e:
        warn(f"Reference {ref_name} used in relation of type {rel_type} does not exist!")
        return False
    else:
        superClasses = list(onto.transitive_objects(ref_type, RDFS.subClassOf))
        if CROW.LocalizedThing not in superClasses:
            warn(f"Reference {ref_name} used in relation of type {rel_type} must be localized! (I.e. subclass of LocalizedThing)")
            return False
        else:
            return True

# %%Build graph
def buildGraph(build_name, onto, recipe_name=None, isBaseBuild=None):
    # %%Load YAML
    base_filename, _ = os.path.splitext(build_name)
    _, base_name = os.path.split(base_filename)

    with open(build_name, "r") as f:
        recipe = yaml.safe_load(f)

    # initialize graph
    graph = pgv.AGraph(strict=False)
    graph.node_attr['style']='filled'

    # add recipe name
    if isBaseBuild is None:
        isBaseBuild = recipe_name is None
    if recipe_name is None:
        recipe_name = recipe["assembly_name"]
    BUILD = Namespace(f"{ONTO_IRI}/{recipe_name}#")
    onto.bind(recipe_name.replace("/", "_"), BUILD)
    recipe_onto_name = BUILD[recipe_name]
    graph.add_node(recipe_name, shape="note", fillcolor="white")
    onto.add((recipe_onto_name, RDF.type, CROW.AssemblyGraph))

    # Add objects
    if "imports" in recipe:
        for sub_build, import_path in recipe["imports"].items():
            g, g_name, _ = buildGraph(import_path, onto, recipe_name + "/" + sub_build)
            sub_name = g_name[g_name.find("/")+ 1:].replace('/', '.')
            for n in g.nodes():
                if str(n) != g_name:
                    new_nodename = f"{sub_build}.{n}"
                    graph.add_node(new_nodename, **n.attr)
                    n = graph.get_node(new_nodename)
                    n.attr["label"] = f"{sub_build}.{n.attr['label']}"
            for e in g.edges():
                u, v = [f"{sub_build}.{w}" for w in tuple(e)]
                graph.add_edge(u, v, **e.attr)
            for triple in onto.triples((getQualifiedName(g_name.replace("/", ".") + f".{g_name}"), None, None)):
                onto.add((recipe_onto_name, triple[1], triple[2]))

    print(f"Parsing build {recipe_name}")
    # Add objects
    if "objects" in recipe:
        for entity, props in recipe["objects"].items():
            node_type = props["type"]
            onto_type = CROW[node_type]
            onto_name = BUILD[entity]
            if checkIValidType(onto_type):
                superClasses = list(onto.transitive_objects(CROW[props["type"]], RDFS.subClassOf))
                if CROW.WorkMaterial in superClasses:
                    onto.add((onto_name, RDF.type, onto_type))
                    onto.add((recipe_onto_name, CROW.usesMaterial, onto_name))
                    if CROW.Workpiece in superClasses:
                        graph.add_node(entity, label=entity + f"\n{props['type']}", shape="box", fillcolor="azure2")
                    elif CROW.Consumable in superClasses:
                        graph.add_node(entity, label=entity + f"\n{props['type']}", shape="box", fillcolor="gray")
                else:
                    warn("Type {node_type} for object {entity} is not a subclass of WorkMaterial!")
            else:
                warn(f"Object class {node_type} for object {entity} not found in Ontology!")

    # Add connections
    if "operations" in recipe:
        for entity, props in recipe["operations"].items():
            node_type = props["type"]
            onto_type = CROW[node_type]
            onto_name = BUILD[entity]
            if checkIValidType(onto_type):
                superClasses = list(onto.transitive_objects(onto_type, RDFS.subClassOf))
                if CROW.AssemblyOperation in superClasses:
                    label = entity + f"\n{node_type[:node_type.index('Connection')]}"
                    graph.add_node(entity, label=label, shape="oval", fillcolor="lavender")
                    onto.add((onto_name, RDF.type, onto_type))
                    onto.add((recipe_onto_name, CROW.hasAssemblyElement, onto_name))
                    if CROW.AssemblyBinaryConnection in superClasses:
                        if CROW.InsertConnection in superClasses:
                            a = props["shaft"]
                            b = props["hole"]
                        else:
                            a = props["provider"]
                            b = props["consumer"]

                        # check onto
                        if checkConnectionObject(entity, a, recipe_name) and checkConnectionObject(entity, b, recipe_name):
                            onto.add((onto_name, CROW.affordanceProvider, getQualifiedName(a, recipe_name)))
                            onto.add((onto_name, CROW.affordanceConsumer, getQualifiedName(b, recipe_name)))
                            graph.add_edge(a, entity, dir="forward", arrowhead="onormal", arrowsize=0.7)
                            graph.add_edge(entity, b, dir="forward", arrowhead="onormal", arrowsize=0.7)
                    else:
                        warn("Cannot process other than binary connections, yet!")
                else:
                    warn(f"Operation type {node_type} for {entity} is not a subclass of AssemblyOperation!")
            else:
                warn(f"Operation type {node_type} for {entity} operation not found in Ontology!")

    # Add relations
    if "relations" in recipe:
        for i, props in enumerate(recipe["relations"]):
            node_type = props["type"]
            onto_type = CROW[node_type]
            if checkIValidType(onto_type):
                name = str(next(onto.triples((onto_type, CROW.representation, None)))[2])
                uname = name + str(i)
                superClasses = list(onto.transitive_objects(onto_type, RDFS.subClassOf))
                if CROW.Relation in superClasses:
                    onto_name = BUILD[uname]
                    graph.add_node(uname, label=name, shape="circle", color="red", fillcolor="indianred1")
                    onto.add((onto_name, RDF.type, onto_type))
                    onto.add((recipe_onto_name, CROW.definesRelation, onto_name))

                    if CROW.UnorderedRelation in superClasses:
                        operations = props["operations"]
                        for op in operations:
                            if checkRelationOperation(node_type, op, recipe_name):
                                onto.add((onto_name, CROW.relatesOperation, getQualifiedName(op, recipe_name)))
                                graph.add_edge(op, uname, color="red")

                    if CROW.RelationWithReference in superClasses:
                        ref = props["reference"]
                        if checkReference(node_type, ref, recipe_name):
                            graph.add_edge(ref, uname, color="red", style="dashed")
                            onto.add((onto_name, CROW.hasSpatialReference, getQualifiedName(ref, recipe_name)))
                else:
                    warn(f"Relation of type {props['type']} is not a subclass of Relation!")
            else:
                warn(f"Relation of type {props['type']} not found in Ontology!")

    # Add order_hints
    if "order_hints" in recipe:
        for i, props in enumerate(recipe["order_hints"]):
            node_type = props["type"]
            onto_type = CROW[node_type]
            if checkIValidType(onto_type):
                name = str(next(onto.triples((CROW[props["type"]], CROW.representation, None)))[2])
                uname = name + str(i)
                superClasses = list(onto.transitive_objects(CROW[props["type"]], RDFS.subClassOf))
                if CROW.Order in superClasses:
                    onto_name = BUILD[node_type + str(i)]
                    graph.add_node(uname, label=name, shape="square", color="darkgreen", fillcolor="honeydew")
                    onto.add((onto_name, RDF.type, onto_type))
                    onto.add((recipe_onto_name, CROW.definesOrder, onto_name))
                    if CROW.SequentialOrder in superClasses:
                        first = props["first"]
                        then = props["then"]
                        if checkOrderOperation(node_type, first, recipe_name) and checkOrderOperation(node_type, then, recipe_name):
                            arrowHead = "open"
                            graph.add_edge(first, uname, color="darkgreen", fillcolor="honeydew", arrowhead=arrowHead, dir="forward")
                            graph.add_edge(uname, then, color="darkgreen", fillcolor="honeydew", arrowhead=arrowHead, dir="forward")
                            onto.add((onto_name, CROW.firstOp, getQualifiedName(first, recipe_name)))
                            onto.add((onto_name, CROW.thenOp,getQualifiedName(then, recipe_name)))
                    else:  # assumes parallel or selection order
                        operations = props["operations"]
                        if all([checkOrderOperation(node_type, op, recipe_name) for op in operations]):
                            if CROW.ParallelOrder in superClasses:
                                arrowHead = "open"
                            elif CROW.SelectionOrder in superClasses:
                                arrowHead = "ediamond"
                            for op in operations:
                                graph.add_edge(uname, op, color="darkgreen", fillcolor="honeydew", arrowhead=arrowHead, dir="forward")
                                onto.add((onto_name, CROW.ordersOperation, getQualifiedName(op, recipe_name)))
                else:
                    warn(f"Unknown order hint type: {node_type}!")

            else:
                warn(f"Order hint of type {props['type']} not found in Ontology!")

    # DeductiveClosure(OWLRL_Semantics).expand(onto)
    # %%Output

    image_file = base_filename + ".png"
    graph.layout(prog='dot')
    graph.draw(image_file, prog="dot")

    return graph, recipe_name, base_filename

# %% Do
g, g_name, base_filename = buildGraph(build_name, onto)

# %% Draw
image_file = base_filename + ".png"
outonto_file = base_filename + ".owl"
image = cv2.imread(image_file)
cv2.imshow(image_file, image)
cv2.waitKey(0)
cv2.destroyAllWindows()

onto.serialize(outonto_file)
