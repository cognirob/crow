from rdflib.namespace import FOAF, RDFS, RDF, OWL, XMLNS, XSD, Namespace
from rdflib.plugins.stores.sparqlstore import SPARQLUpdateStore, Store, _node_to_sparql

import rdflib
from rdflib import BNode
from crow_ontology.crowracle_client import CrowtologyClient
from rdflib.plugins.sparql.processor import prepareQuery
from time import time
import timeit
import numpy as np
import warnings
import datetime as dt
from uuid import uuid4


CROW = Namespace(f'http://imitrob.ciirc.cvut.cz/ontologies/crow#')


def speed_test(func, repeat=5):

    def wrapped(in_mean=0, in_median=0, in_min=0, in_max=0):
        start = time()
        _ = func()
        prerun_time = time() - start

        n = int(np.ceil(1 / prerun_time))
        if n == 1:
            warnings.warn(f"The function runtime is very long! ({prerun_time} seconds)")
        results = timeit.repeat(stmt=func, repeat=repeat, number=n)
        results = np.array(results) / n

        r_mean = np.mean(results) - in_mean
        r_median = np.median(results) - in_median
        r_min = np.min(results) - in_min
        r_max = np.max(results) - in_max
        print(f"Function '{func.__name__}' runtime:\nmean =\t {r_mean:0.4f} s \u00B1 {np.std(results):0.3f}\nmedian =\t {r_median:0.4f} s\nmin = {r_min:0.4f} s / max = {r_max:0.4f} s")
        return r_mean, r_median, r_min, r_max

    return wrapped


def enabled(is_enabled):

    if is_enabled:
        return lambda func: func
    else:
        return lambda func: func.__name__


ONTO_IRI = "http://imitrob.ciirc.cvut.cz/ontologies/crow"


class SpeedTester():
    CROW_OBJECTS = ["cube_holes", "sphere_holes", "wheel", "wafer", "peg", "screw", "screw_driver", "hammer", "wrench"]
    CROW_TEMPLATES = ["CUBE", "SPHERE", "WHEEL", "WAFER", "PEG", "SCREW", "SCREW_DRIVER", "HAMMER", "WRENCH"]
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
        return object_name, location, size, uuid, timestamp, template, adder_id

    def add_object(self):
        object_name, location, size, uuid, timestamp, template, adder_id = self.get_me_an_object()
        self.crowracle.add_detected_object(object_name, location, size, uuid, timestamp, template, adder_id)
        return self.crowracle.CROW[object_name + '_od_'+str(adder_id)]

    @enabled(is_enabled=False)
    def test_add_object(self):
        self.crowracle.reset_database()
        print("Testing if add_detected_object is working...")
        objs = [self.add_object() for i in range(5)]
        for obj in objs:
            current_objects = self.crowracle.getTangibleObjects()
            assert obj in current_objects, "Object missing from database after addition!"
        print("Success!")
        self.crowracle.reset_database()

        self.run_prep_and_func(self.get_me_an_object, self.crowracle.add_detected_object)
        self.crowracle.reset_database()

    @enabled(is_enabled=False)
    def test_delete_object(self):
        self.crowracle.reset_database()
        print("Testing if delete_object is working...")
        objs = [self.add_object() for i in range(5)]
        for obj in objs:
            current_objects = self.crowracle.getTangibleObjects()
            assert obj in current_objects, "Object missing from database. Maybe error in add_object?"
            self.crowracle.delete_object(obj)
            current_objects = self.crowracle.getTangibleObjects()
            # print(current_objects, obj)
            assert obj not in current_objects, "Object still exists in database after deletion!"
        print("Success!")

        self.run_prep_and_func(self.add_object, self.crowracle.delete_object)
        self.crowracle.reset_database()

    @enabled(is_enabled=True)
    def test_update_object(self):
        self.crowracle.reset_database()
        print("Testing if update_object is working...")
        objs = [self.add_object() for i in range(5)]
        # for obj in objs:
        #     current_objects = self.crowracle.getTangibleObjects()
        #     assert obj in current_objects, "Object missing from database. Maybe error in add_object?"
        #     self.crowracle.delete_object(obj)
        #     current_objects = self.crowracle.getTangibleObjects()
        #     # print(current_objects, obj)
        #     assert obj not in current_objects, "Object still exists in database after deletion!"
        print("Success!")

        # self.run_prep_and_func(self.add_object, self.crowracle.delete_object)
        # self.crowracle.reset_database()

    @enabled(is_enabled=False)
    def test_disable_enable_object(self):
        self.crowracle.reset_database()
        objs = [self.add_object() for i in range(5)]
        print("Testing if disable and enable is working...")
        for obj in objs:
            current_objects = self.crowracle.getTangibleObjects()
            assert obj in current_objects, "Object missing from database. Maybe error in add_object?"
            self.crowracle.disable_object(obj)
            current_objects = self.crowracle.getTangibleObjects()
            assert obj not in current_objects, "Object still returned by getTangibleObjects after disabling!"
            current_objects = self.crowracle.getTangibleObjects_disabled_nocls()
            assert obj in current_objects, "Object disappeared from database after disabling!"
            self.crowracle.enable_object(obj)
            current_objects = self.crowracle.getTangibleObjects()
            assert obj in current_objects, "Object not returned by getTangibleObjects after enabling!"
        print("Success!")
        self.crowracle.reset_database()

        print("Testing disable:")
        self.run_prep_and_func(self.add_object, self.crowracle.disable_object)
        print("Testing enable:")
        def add_and_disable():
            obj = self.add_object()
            self.crowracle.disable_object(obj)
            return obj
        self.run_prep_and_func(add_and_disable, self.crowracle.enable_object)
        self.crowracle.reset_database()

    def run_tests(self):
        for test_function in [getattr(self, elm) for elm in dir(self) if elm.startswith("test_")]:
            print("*" * self.SEPLEN)
            if type(test_function) is str:
                print(f"Skipping {test_function}...")
            else:
                print(f"Running test function {test_function.__name__}...")
                test_function()



# %% CROWTOLOGY

def main(args=[]):
    tester = SpeedTester()
    tester.run_tests()

if __name__ == '__main__':
    main()