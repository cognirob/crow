#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Aug 26 16:37:08 2021

@author: syxtreme
"""
# %% imports
from rdflib.namespace import FOAF, RDFS, RDF, OWL, XMLNS, XSD, Namespace
from rdflib.plugins.stores.sparqlstore import SPARQLUpdateStore, Store, _node_to_sparql

import rdflib
from rdflib import BNode, Literal, URIRef
from crow_ontology.crowracle_client import CrowtologyClient
from rdflib.plugins.sparql.processor import prepareQuery
CROW = Namespace(f'http://imitrob.ciirc.cvut.cz/ontologies/crow#')
from uuid import uuid4
import datetime as dt
from knowl import DBConfig, OntologyAPI


# %% CROWTOLOGY

crowracle = CrowtologyClient()
onto = crowracle.onto
onto_namespaces = crowracle.onto.namespaces
if "crow" not in onto_namespaces.keys():
    onto_namespaces["crow"] = crowracle.CROW
replacement_ns = {}
for ns, uri in onto_namespaces.items():
    if ns == "base":
        continue
    uri = str(uri)
    replacement_ns[uri] = ns + ":"
    if uri.endswith("#"):
        replacement_ns[uri[:-1] + "/"] = ns + ":"
    if uri.endswith("/"):
        replacement_ns[uri[:-1] + "#"] = ns + ":"

# %% DIRECT DB API
config = DBConfig(**{
    "username": "root",
    "password": "vzducholod",
    "host": "localhost",
    "port": "3030",
    "store": "fuseki",
    "fuseki_path": "/home/imitrob/packages/apache-jena-fuseki-4.2.0",
    "database": "onto",
    "baseURL": "http://imitrob.ciirc.cvut.cz/ontologies/crow",
})
onto = OntologyAPI(config)

# %% NLP test
query = """
PREFIX crow:    <http://imitrob.ciirc.cvut.cz/ontologies/crow#>
PREFIX rdfs: <http://www.w3.org/2000/01/rdf-schema#>
PREFIX rdf: <http://www.w3.org/1999/02/22-rdf-syntax-ns#>
PREFIX owl: <http://www.w3.org/2002/07/owl#>

SELECT DISTINCT ?name

WHERE {
    BIND(<http://imitrob.ciirc.cvut.cz/ontologies/crow#Cube> AS ?obj)
    ?obj a ?cls .
    BIND(IF(?cls = crow:NamedColor || ?cls = owl:Class, ?obj, ?cls) AS ?target)
  	?target crow:hasNlpNameCZ ?name .
}"""
result = onto.query(query)
print(list(result))
# %% Add object
ONTO_IRI = "http://imitrob.ciirc.cvut.cz/ontologies/crow"
object_name = "sphere_holes"
location = [1, 2, 3]
size = [0.05, 0.05, 0.05]
uuid = str(uuid4())
timestamp = dt.datetime.now().strftime('%Y-%m-%dT%H:%M:%SZ')
template = crowracle.CROW.SPHERE
adder_id = 2

individual_name = object_name + '_od_'+str(adder_id)
PART = Namespace(f"{ONTO_IRI}/{individual_name}#")


query = """PREFIX rdf: <http://www.w3.org/1999/02/22-rdf-syntax-ns#>
prefix owl: <http://www.w3.org/2002/07/owl#>
prefix rdfs: <http://www.w3.org/2000/01/rdf-schema#>
prefix crow: <http://imitrob.ciirc.cvut.cz/ontologies/crow#>

INSERT {{ 
  {individual} ?prop ?value .
  {individual} crow:hasAbsoluteLocation {loc_name} .
  {individual} crow:hasPclDimensions {pcl_name} .
  {loc_name} a crow:xyzAbsoluteLocation .
  {loc_name} crow:x {loc_x} .
  {loc_name} crow:y {loc_y} .
  {loc_name} crow:z {loc_z} .
  {pcl_name} a crow:xyzPclDimensions .
  {pcl_name} crow:x {pcl_x} .
  {pcl_name} crow:y {pcl_y} .
  {pcl_name} crow:z {pcl_z} .
  {individual} crow:hasId {adder_id} .
  {individual} crow:hasUuid {uuid} .
  {individual} crow:hasTimestamp {stamp} .

}}

WHERE {{
  {template} ?prop ?value .
  FILTER NOT EXISTS {{ ?any crow:hasAbsoluteLocation ?value }}
  FILTER NOT EXISTS {{ ?any crow:hasPclDimensions ?value }}
}}
""".format(**{
    "individual": CROW[individual_name].n3(),
    "loc_name": PART.xyzAbsoluteLocation.n3(),
    "loc_x": Literal(location[0], datatype=XSD.float).n3(),
    "loc_y": Literal(location[1], datatype=XSD.float).n3(),
    "loc_z": Literal(location[2], datatype=XSD.float).n3(),
    "pcl_name": PART.hasPclDimensions.n3(),
    "pcl_x": Literal(size[0], datatype=XSD.float).n3(),
    "pcl_y": Literal(size[1], datatype=XSD.float).n3(),
    "pcl_z": Literal(size[2], datatype=XSD.float).n3(),
    "template": template.n3(),
    "adder_id": Literal('od_'+str(adder_id), datatype=XSD.string).n3(),
    "uuid": Literal(uuid, datatype=XSD.string).n3(),
    "stamp": Literal(timestamp, datatype=XSD.dateTimeStamp).n3()
})
print(query)

# %% Add object NO TEMPLATE
ONTO_IRI = "http://imitrob.ciirc.cvut.cz/ontologies/crow"
object_name = "sphere_holes"
location = [1, 2, 3]
size = [0.05, 0.05, 0.05]
uuid = str(uuid4())
timestamp = dt.datetime.now().strftime('%Y-%m-%dT%H:%M:%SZ')
adder_id = 2

individual_name = object_name + '_od_'+str(adder_id)
PART = Namespace(f"{ONTO_IRI}/{individual_name}#")


query = """
INSERT {
    ?individual ?prop ?value .
    ?individual crow:hasAbsoluteLocation ?loc_name .
    ?individual crow:hasPclDimensions ?pcl_name .
    ?loc_name a crow:xyzAbsoluteLocation .
    ?loc_name crow:x ?loc_x .
    ?loc_name crow:y ?loc_y .
    ?loc_name crow:z ?loc_z .
    ?pcl_name a crow:xyzPclDimensions .
    ?pcl_name crow:x ?pcl_x .
    ?pcl_name crow:y ?pcl_y .
    ?pcl_name crow:z ?pcl_z .
    ?individual crow:hasId ?adder_id .
    ?individual crow:hasUuid ?uuid .
    ?individual crow:hasTimestamp ?stamp .
}

WHERE {
    ?template ?prop ?value ;
            crow:hasDetectorName ?det_name .
    FILTER NOT EXISTS { ?any crow:hasAbsoluteLocation ?value }
    FILTER NOT EXISTS { ?any crow:hasPclDimensions ?value }
}"""
initBindings = {
    "individual": CROW[individual_name],
    "loc_name": PART.xyzAbsoluteLocation,
    "loc_x": Literal(location[0], datatype=XSD.float),
    "loc_y": Literal(location[1], datatype=XSD.float),
    "loc_z": Literal(location[2], datatype=XSD.float),
    "pcl_name": PART.hasPclDimensions,
    "pcl_x": Literal(size[0], datatype=XSD.float),
    "pcl_y": Literal(size[1], datatype=XSD.float),
    "pcl_z": Literal(size[2], datatype=XSD.float),
    "det_name": Literal(object_name, datatype=XSD.string),
    "adder_id": Literal('od_'+str(adder_id), datatype=XSD.string),
    "uuid": Literal(uuid, datatype=XSD.string),
    "stamp": Literal(timestamp, datatype=XSD.dateTimeStamp)

}
print(query)
onto.update(query, initBindings=initBindings)
# %% Filter by uuid
# uuids = ["655ae8d7-0cc3-46a6-9355-0f2f5a02f9f1"]
uuids = ["ecc3cd32-65ce-4a44-93f6-79ba73634ccc", "fake-uuid0", "655ae8d7-0cc3-46a6-9355-0f2f5a02f9f1"]
# Return tuple (object, uuid) for uuids that exist in the DB
query = """SELECT DISTINCT ?obj ?uuid

    WHERE {{
    # ?obj a ?cls .
    # ?cls rdfs:subClassOf+ crow:TangibleObject .
    ?obj crow:hasUuid ?uuid .
    FILTER EXISTS {{ ?obj crow:hasTimestamp ?any }}
    FILTER (?uuid IN ({list_of_uuid}))
               
}}""".format(list_of_uuid=",".join([f"'{u}'" for u in uuids]))
print(query)
# %% Reutrn also not in DB
"""      {
    	?obj a ?cls .
        ?cls rdfs:subClassOf+ crow:TangibleObject .
        ?obj crow:hasUuid ?uuid .
        FILTER EXISTS { ?obj crow:hasTimestamp ?any }
        BIND ( IF (?uuid IN ("ecc3cd32-65ce-4a44-93f6-79ba73634ccc", "655ae8d7-0c4c3-46a6-9355-0f2f5a02f9f1"),
                    True,
                    False
      		) AS ?was_matched)
    	BIND ( IF (?was_matched, ?uuid, 1/0) as ?matched_uuid)
        OPTIONAL {
            ?template a ?cls .
      		FILTER (! ?was_matched )
            FILTER NOT EXISTS { ?template crow:hasTimestamp ?any_other }
            FILTER NOT EXISTS { ?template crow:hasUuid ?any_uuid . }
    	}
"""
# %% DELETE
object_name = "sphere_holes"
adder_id = 2

individual_name = object_name + '_od_'+str(adder_id)
individual = CROW[individual_name]
query = """DELETE {{
  	?s ?p ?o .
}}
WHERE {{
    ?s ?p ?o .
 	FILTER (?s = {individual} || ?p = {individual})command_main_buffer
}}""".format(individual=individual.n3())
print(query)
# %% Fuseki
def my_bnode_ext(node):

   if isinstance(node, BNode):
       return '<bnode:b%s>' % node
   return _node_to_sparql(node)

store = SPARQLUpdateStore(queryEndpoint="http://localhost:4242/onto/sparql", update_endpoint="http://localhost:4242/onto/update", context_aware=True, postAsEncoded=False, node_to_sparql=my_bnode_ext)
query_endpoint = 'http://localhost:3030/onto/query'
update_endpoint = 'http://localhost:3030/onto/update'
store.method = 'POST'
store.open((query_endpoint, update_endpoint))

g = rdflib.Graph(store, identifier="http://imitrob.ciirc.cvut.cz/ontologies/crow")

# %%
tmpGraph = rdflib.Graph()
tmpGraph.parse("/home/syxtreme/crow/vision_ws/src/crow/crow_ontology/data/onto_draft_03.owl")
g += tmpGraph