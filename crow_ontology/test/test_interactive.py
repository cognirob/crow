from rdflib.namespace import FOAF, RDFS, RDF, OWL, XMLNS, XSD, Namespace
from rdflib.plugins.stores.sparqlstore import SPARQLUpdateStore, Store, _node_to_sparql

import rdflib
from rdflib import BNode, URIRef, Literal
from crow_ontology.crowracle_client import CrowtologyClient
from rdflib.plugins.sparql.processor import prepareQuery
from time import time
import timeit
import numpy as np
import warnings
import datetime as dt
from uuid import uuid4


CROW = Namespace(f'http://imitrob.ciirc.cvut.cz/ontologies/crow#')

ONTO_IRI = "http://imitrob.ciirc.cvut.cz/ontologies/crow"


class SpeedTester():
    CROW_OBJECTS = ["cube_holes", "sphere_holes", "wheel", "wafer", "peg_screw", "screw_round", "screwdriver", "hammer", "wrench"]
    CROW_TEMPLATES = ["CUBE", "SPHERE", "WHEEL", "WAFER", "PEG", "SCREW", "FLATSCREWDRIVER", "HAMMER", "WRENCH"]
    SEPLEN = 30

    def __init__(self):
        self.crowracle = CrowtologyClient()
        self.onto = self.crowracle.onto
        onto_namespaces = self.crowracle.onto.namespaces
        if "crow" not in onto_namespaces.keys():
            onto_namespaces["crow"] = self.crowracle.CROW
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

    def run_prep_and_func(self, prep_data_function, main_funcion):
        self.SEPLEN
        print("#" * self.SEPLEN)
        print("Testing the runtime of the data preparation function (subtract this from the total runtime)...")
        prep_func = speed_test(prep_data_function)
        r_mean, r_median, r_min, r_max = prep_func()
        print("-" * self.SEPLEN)
        print("Running the main test function...")
        @speed_test
        def run_function():
            data = prep_data_function()
            if type(data) is not tuple:
                main_funcion(data)
            else:
                main_funcion(*data)
        run_function(r_mean, r_median, r_min, r_max)
        print("#" * self.SEPLEN)

    def get_me_an_object(self):
        choice = np.random.choice(len(self.CROW_OBJECTS))
        object_name = self.CROW_OBJECTS[choice]
        location = np.random.randn(3) * 5
        size = (np.random.rand(3) * 2 + 0.5) / 10
        uuid = str(uuid4())
        timestamp = dt.datetime.now().strftime('%Y-%m-%dT%H:%M:%SZ')
        template = self.crowracle.CROW[self.CROW_TEMPLATES[choice]]
        adder_id = np.random.randint(0, 2**32)
        tracked = True
        return object_name, location, size, uuid, timestamp, template, adder_id, tracked

    def get_no_template(self):
        object_name, location, size, uuid, timestamp, template, adder_id, tracked = self.get_me_an_object()
        return object_name, location, size, uuid, timestamp, adder_id, tracked

    def get_me_an_object_around(self, x=0, y=0, z=0, sigma=0.2):
        choice = np.random.choice(len(self.CROW_OBJECTS))
        object_name = self.CROW_OBJECTS[choice]
        location = np.r_[np.random.randn(2) * sigma, np.random.rand() * 0.03] + np.r_[x, y, z]
        size = (np.random.rand(3) * 2 + 0.5) / 10
        uuid = str(uuid4())
        timestamp = dt.datetime.now().strftime('%Y-%m-%dT%H:%M:%SZ')
        template = self.crowracle.CROW[self.CROW_TEMPLATES[choice]]
        adder_id = np.random.randint(0, 2**32)
        tracked = True
        return object_name, location, size, uuid, timestamp, template, adder_id, tracked

    def add_object(self):
        object_name, location, size, uuid, timestamp, template, adder_id, tracked = self.get_me_an_object()
        self.crowracle.add_detected_object(object_name, location, size, uuid, timestamp, template, adder_id, tracked)
        return self.crowracle.CROW[object_name + '_od_'+str(adder_id)]

    def add_and_gen_update(self):
        obj = self.add_object()
        update_location = np.random.randn(3) * 5
        update_size = (np.random.rand(3) * 2 + 0.5) / 10
        update_timestamp = dt.datetime.now().strftime('%Y-%m-%dT%H:%M:%SZ')
        update_tracked = bool(np.random.randint(0, 2))
        return obj, update_location, update_size, update_timestamp, update_tracked


# %% CROWTOLOGY

def main(args=[]):
    pass

tester = SpeedTester()
onto = tester.onto
crw = tester.crowracle

# %%  SUPER LARGE UPDATE
crw.reset_database()

batch_update = [tester.add_and_gen_update() for i in range(10)]
batch_add = [tester.get_no_template() for i in range(10)]

inserts = ""
deletes = ""
wheres = ""
union_started = False

for i, obj in enumerate(batch_update):
    object, location, size, timestamp, tracked = obj
    object = URIRef(object).n3()
    new_stamp = Literal(timestamp, datatype=XSD.dateTimeStamp).n3()
    new_x = Literal(location[0], datatype=XSD.float).n3()
    new_y = Literal(location[1], datatype=XSD.float).n3()
    new_z = Literal(location[2], datatype=XSD.float).n3()
    new_pcl_x = Literal(size[0], datatype=XSD.float).n3()
    new_pcl_y = Literal(size[1], datatype=XSD.float).n3()
    new_pcl_z = Literal(size[2], datatype=XSD.float).n3()
    tracked = Literal(tracked, datatype=XSD.boolean).n3()

    deletes += f"""
        {object} crow:hasTimestamp ?old_stamp{str(i)} .
        ?loc{str(i)} crow:x ?old_x{str(i)} .
        ?loc{str(i)} crow:y ?old_y{str(i)} .
        ?loc{str(i)} crow:z ?old_z{str(i)} .
        ?pcl{str(i)} crow:x ?old_pcl_x{str(i)} .
        ?pcl{str(i)} crow:y ?old_pcl_y{str(i)} .
        ?pcl{str(i)} crow:z ?old_pcl_z{str(i)} .
        {object} crow:isTracked ?old_tracked{str(i)} .
    """
    inserts += f"""
        {object} crow:hasTimestamp {new_stamp} .
        ?loc{str(i)} crow:x {new_x} .
        ?loc{str(i)} crow:y {new_y} .
        ?loc{str(i)} crow:z {new_z} .
        ?pcl{str(i)} crow:x {new_pcl_x} .
        ?pcl{str(i)} crow:y {new_pcl_y} .
        ?pcl{str(i)} crow:z {new_pcl_z} .
        {object} crow:isTracked {tracked} .
    """
    if union_started:
        wheres += "UNION {"
    else:
        wheres += "{"
        union_started = True
    wheres += f"""
            {object} crow:hasTimestamp ?old_stamp{str(i)} .
            {object} crow:hasAbsoluteLocation ?loc{str(i)} .
            {object} crow:hasPclDimensions ?pcl{str(i)} .
            ?loc{str(i)} crow:x ?old_x{str(i)} .
            ?loc{str(i)} crow:y ?old_y{str(i)} .
            ?loc{str(i)} crow:z ?old_z{str(i)} .
            ?pcl{str(i)} crow:x ?old_pcl_x{str(i)} .
            ?pcl{str(i)} crow:y ?old_pcl_y{str(i)} .
            ?pcl{str(i)} crow:z ?old_pcl_z{str(i)} .
            {object} crow:isTracked ?old_tracked{str(i)} .
        }}
    """

already_template = []
for i, obj in enumerate(batch_add):
    object_name, location, size, uuid, timestamp, adder_id, tracked = obj
    individual_name = object_name + '_od_'+str(adder_id)
    individual = CROW[individual_name].n3()
    PART = Namespace(f"{ONTO_IRI}/{individual_name}#")
    loc = PART.xyzAbsoluteLocation.n3()
    pcl = PART.hasPclDimensions.n3()
    inserts += f"""
        {individual} ?prop_{object_name} ?value_{object_name} .
        {individual} crow:hasAbsoluteLocation {loc} .
        {individual} crow:hasPclDimensions {pcl} .
        {loc} a crow:xyzAbsoluteLocation .
        {loc} crow:x {Literal(location[0], datatype=XSD.float).n3()} .
        {loc} crow:y {Literal(location[1], datatype=XSD.float).n3()} .
        {loc} crow:z {Literal(location[2], datatype=XSD.float).n3()} .
        {pcl} a crow:xyzPclDimensions .
        {pcl} crow:x {Literal(size[0], datatype=XSD.float).n3()} .
        {pcl} crow:y {Literal(size[1], datatype=XSD.float).n3()} .
        {pcl} crow:z {Literal(size[2], datatype=XSD.float).n3()} .
        {individual} crow:hasId {Literal('od_'+str(adder_id), datatype=XSD.string).n3()} .
        {individual} crow:hasUuid {Literal(uuid, datatype=XSD.string).n3()} .
        {individual} crow:hasTimestamp {Literal(timestamp, datatype=XSD.dateTimeStamp).n3()} .
        {individual} crow:isTracked {Literal(tracked, datatype=XSD.boolean).n3()} .
    """
    if object_name not in already_template:
        already_template.append(object_name)
        if union_started:
            wheres += "UNION {"
        else:
            wheres += "{"
            union_started = True        
        wheres += f"""
                ?template_{object_name} ?prop_{object_name} ?value_{object_name} ;
                        crow:hasDetectorName {Literal(object_name, datatype=XSD.string).n3()} .
                FILTER NOT EXISTS {{ ?template_{object_name} crow:hasUuid ?uuid_any_{object_name} }}
                MINUS {{ ?template_{object_name} crow:hasAbsoluteLocation|crow:hasPclDimensions ?value_{object_name} }}
            }}
        """
    
query = f"""PREFIX rdf: <http://www.w3.org/1999/02/22-rdf-syntax-ns#>
    prefix owl: <http://www.w3.org/2002/07/owl#>
    prefix rdfs: <http://www.w3.org/2000/01/rdf-schema#>
    prefix crow: <http://imitrob.ciirc.cvut.cz/ontologies/crow#>
    DELETE {{
        {deletes}
    }}\n"""
query += f"""INSERT {{
        {inserts}
    }}\n"""
query += f"""WHERE {{
    {wheres}
}}"""
print(query)
# %%

onto.update(query)

