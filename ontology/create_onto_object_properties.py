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
#parser.add_argument("properties_file")
parser.add_argument("--onto_file", "-o", default="ontology/onto_draft_01.owl")
args = parser.parse_args()

# %%Initialization
#properties_file = args.properties_file
properties_file = "ontology/assembly/properties.yaml"
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

def getQualifiedName(obj_name, build_name=None):
    obj_name, BUILD = getBaseNameAndNS(obj_name, build_name)
    return BUILD[obj_name]

def checkisValid(onto_entity):
    if len(list(onto.triples((onto_entity, RDF.type, None)))) > 0:
        return True
    else:
        return False

def checkIsValidDomain(onto_name, domain):
    superClasses = list(onto.transitive_objects(onto_name, RDFS.subClassOf))
    onto_name_type = list(onto.triples((onto_name, RDF.type, None)))[0][2]
    superClassesType = list(onto.transitive_objects(onto_name_type, RDFS.subClassOf))
    if any([d in superClasses for d in domain]):
        return True
    elif any([d in superClassesType for d in domain]):
        return True
    else:
        return False

def add_parsed_property(ns, onto_name, prop, value):
    onto_property = ns[0][prop]
    if checkisValid(onto_property): #check if property is defined in onto
        prop_domain_list = list(onto.triples((onto_property, RDFS.domain, None)))
        prop_domain = []
        for pd in prop_domain_list:
            prop_domain.append(pd[2])
        prop_range = list(onto.triples((onto_property, RDFS.range, None)))[0][2]

        if prop == 'isEnabled':
            prop_name = ns[0]['enabled']
            onto.add((prop_name, RDF.type, prop_range))
            onto.add((prop_name, CROW.isBool, Literal(value, datatype=XSD.boolean)))
            onto.add((onto_name, onto_property, prop_name))
        elif prop_range in [CROW.Location, CROW.Vector]:
            if value is None:
                value = [None]*3
                prop_name = ns[0][prop.replace('has','xyz')]
            else:
                onto_base_name, _ = getBaseNameAndNS(onto_name)
                PART = Namespace(f"{ONTO_IRI}/{onto_base_name}#") #ns for each object (/Cube#)
                prop_name = PART[prop.replace('has','xyz')]
            onto.add((prop_name, RDF.type, prop_range))
            onto.add((prop_name, CROW.x, Literal(value[0], datatype=XSD.float)))
            onto.add((prop_name, CROW.y, Literal(value[1], datatype=XSD.float)))
            onto.add((prop_name, CROW.z, Literal(value[2], datatype=XSD.float)))
            onto.add((onto_name, onto_property, prop_name))
            #print(str(prop_name+'+'+RDF.type+'+'+prop_range)+'\n')
            #print(str(onto_name+'+'+onto_property+'+'+prop_name)+'\n')

        else:
            if checkIsValidDomain(onto_name, prop_domain): #check domain
                if checkisValid(ns[0][value]): #check if value is resource or not
                    onto_value = ns[0][value]
                    onto.add((onto_name, onto_property, onto_value))
                    #print(str(onto_name+'+'+onto_property+'+'+onto_value)+'\n')
                elif len(ns) > 1 and checkisValid(ns[1][value]): #check if value is resource or not
                    onto_value = ns[1][value]
                    onto.add((onto_name, onto_property, onto_value))
                    #print(str(onto_name+'+'+onto_property+'+'+onto_value)+'\n')
                else:
                    onto.add((onto_name, onto_property, Literal(value, datatype=prop_range)))
                    #print(str(onto_name+'+'+onto_property+'+'+str(value)+'+'+prop_range)+'\n')
            else:
                warn(f"Class {onto_name} is not valid for domain {prop_domain} of a property {onto_property}!")
    else:
        warn(f"Object property {prop} not found in Ontology!")

# %%Build graph
def buildGraph(properties_file, onto, recipe_name=None):
    # %%Load YAML
    with open(properties_file, "r") as f:
        property_defs = yaml.safe_load(f)

    # Add objects
    print(f"Parsing {properties_file}\n")
    # Add objects
    if "objects" in property_defs:
        for entity, props in property_defs["objects"].items():
            BUILD = Namespace(f"{ONTO_IRI}/{entity}#") #ns for each object (/Cube#)
            onto.bind(entity.replace("/", "_"), BUILD)
            onto_name = CROW[entity.upper()]
            if checkisValid(CROW[entity]):
                onto.add((onto_name, RDF.type, CROW[entity])) #add Object-hasObjectPart-ObjectPart
                for prop, value in props.items():
                    if isinstance(value, list) and isinstance(value[0], dict): #hasObjectPart is a list of one or more parts
                        for i in range(len(value)):
                            part_name = BUILD[value[i]['name']]
                            part_type = CROW[value[i]['type']]
                            onto.add((part_name, RDF.type, part_type)) #add ObjectPart to onto
                            onto.add((onto_name, CROW[prop], part_name)) #add Object-hasObjectPart-ObjectPart
                            #print(str(part_name+'+'+part_type)+'\n')
                            #print(str(onto_name+'+'+CROW[prop]+'+'+part_name)+'\n')
                        for i in range(len(value)):
                            part_name = BUILD[value[i]['name']]
                            part_type = CROW[value[i]['type']]
                            for part_prop, part_value in value[i].items(): 
                                if part_prop not in ['name', 'type']:
                                    add_parsed_property([CROW, BUILD], part_name, part_prop, part_value) #add predicate to ObjectPart
                    else:
                        add_parsed_property([CROW], onto_name, prop, value) #add predicate to Object

    # Add connections
    if "operations" in property_defs:
        for entity, props in property_defs["operations"].items():
            node_type = props["type"]
            onto_type = CROW[node_type]
            onto_name = BUILD[entity]
            if checkIValidType(onto_type):
                superClasses = list(onto.transitive_objects(onto_type, RDFS.subClassOf))
                if CROW.AssemblyOperation in superClasses:
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
                    else:
                        warn("Cannot process other than binary connections, yet!")
                else:
                    warn(f"Operation type {node_type} for {entity} is not a subclass of AssemblyOperation!")
            else:
                warn(f"Operation type {node_type} for {entity} operation not found in Ontology!")

    # Add relations
    if "relations" in property_defs:
        for i, props in enumerate(property_defs["relations"]):
            node_type = props["type"]
            onto_type = CROW[node_type]
            if checkIValidType(onto_type):
                name = str(next(onto.triples((onto_type, CROW.representation, None)))[2])
                uname = name + str(i)
                superClasses = list(onto.transitive_objects(onto_type, RDFS.subClassOf))
                if CROW.Relation in superClasses:
                    onto_name = BUILD[uname]
                    onto.add((onto_name, RDF.type, onto_type))
                    onto.add((recipe_onto_name, CROW.definesRelation, onto_name))

                    if CROW.UnorderedRelation in superClasses:
                        operations = props["operations"]
                        for op in operations:
                            if checkRelationOperation(node_type, op, recipe_name):
                                onto.add((onto_name, CROW.relatesOperation, getQualifiedName(op, recipe_name)))

                    if CROW.RelationWithReference in superClasses:
                        ref = props["reference"]
                        if checkReference(node_type, ref, recipe_name):
                            onto.add((onto_name, CROW.hasSpatialReference, getQualifiedName(ref, recipe_name)))
                else:
                    warn(f"Relation of type {props['type']} is not a subclass of Relation!")
            else:
                warn(f"Relation of type {props['type']} not found in Ontology!")

    # Add order_hints
    if "order_hints" in property_defs:
        for i, props in enumerate(property_defs["order_hints"]):
            node_type = props["type"]
            onto_type = CROW[node_type]
            if checkIValidType(onto_type):
                name = str(next(onto.triples((CROW[props["type"]], CROW.representation, None)))[2])
                uname = name + str(i)
                superClasses = list(onto.transitive_objects(CROW[props["type"]], RDFS.subClassOf))
                if CROW.Order in superClasses:
                    onto_name = BUILD[node_type + str(i)]
                    onto.add((onto_name, RDF.type, onto_type))
                    onto.add((recipe_onto_name, CROW.definesOrder, onto_name))
                    if CROW.SequentialOrder in superClasses:
                        first = props["first"]
                        then = props["then"]
                        if checkOrderOperation(node_type, first, recipe_name) and checkOrderOperation(node_type, then, recipe_name):
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
                                onto.add((onto_name, CROW.ordersOperation, getQualifiedName(op, recipe_name)))
                else:
                    warn(f"Unknown order hint type: {node_type}!")

            else:
                warn(f"Order hint of type {props['type']} not found in Ontology!")

    # DeductiveClosure(OWLRL_Semantics).expand(onto)
    # %%Output

# %% Do
buildGraph(properties_file, onto)

# %% Draw
outonto_file = "ontology/onto_draft_01_debug.owl"

onto.serialize(outonto_file)
