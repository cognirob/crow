# -*- coding: utf-8 -*-
"""
Created on 1.8.2021

@author: Karla Stepanova, CIIRC CTU
"""
import rdflib
import numpy as np
import networkx as nx
import yaml
import pickle
import os
import argparse
import re
import matplotlib.pyplot as plt
import pygraphviz
from networkx.drawing.nx_agraph import graphviz_layout
from collections import Counter

# %%ArgParser
parser = argparse.ArgumentParser()
parser.add_argument("build_name")
parser.add_argument("--onto_file", "-o", default="../ontology/onto_draft_02.owl")
# args = parser.parse_args(["build_dog.yaml"])
args = parser.parse_args(["build_snake.yaml", "-o", "../ontology/onto_draft_02.owl"])
# os.chdir(r".\code\crow\ontology\assembly")
# os.chdir(r".\assembly")
# args = parser.parse_args()

# %%Initialization
# build_name = "build_dog.yaml"
build_name = args.build_name
# onto_file = "onto_draft_03.owl"
# onto_file = "dog_build.owl"
onto_file = args.onto_file
print(onto_file)
split_name_re = re.compile(r"([\w\/]+)\.?")
# split_name_re = re.compile(r".*\.(\w*)\.(\w*)")

# %%Load onto
ONTO_IRI = "http://www.semanticweb.org/crow/ontologies/2019/6/onto_draft_01"
# CROW = Namespace(f"{ONTO_IRI}#")

onto = rdflib.Graph()
onto.load(onto_file)


# %%Build graph
def build_graph(build_name, onto, recipe_name=None, isBaseBuild=None):
    # %%Load YAML
    base_filename, _ = os.path.splitext(build_name)
    _, base_name = os.path.split(base_filename)


    # if os.path.exists('mydirectory/myfile.png')
    with open(build_name, "r") as f:
        recipe = yaml.safe_load(f)

    G = nx.DiGraph()
    # Add recipe name
    assembly_name = recipe["assembly_name"]

    # Add nodes - first for each object one node
    Gin = nx.DiGraph()
    G.add_node(0, parts_names=[None], parts_type=[None], prob=1,
               weight=1, graph=Gin)
    if "objects" in recipe:
        for i, (entity, props) in enumerate(recipe["objects"].items()):
            node_type = props["type"]
            Gin = nx.DiGraph()
            Gin.add_node(entity, part_type=node_type)
            G.add_node(i+1, parts_names=[entity], parts_type=[node_type], prob=0,
                       weight=1, graph=Gin)
            G.add_edge(0, i+1, weight=1, prob=1 / (len(recipe) + 1), action='move',
                       object=node_type)
            # G.add_node(i, probs=[1 / (len(recipe) + 1)], graph=Gin)
    # now add nodes in the way that for each current node it checks if there is an operation with the object in the node and if yes,
    # it adds the new part to the object
    # it is checked on individual levels - in each level, one object should be added
    # TODO improve the level thing - this was done just to not change the size of dictionary during iterations (that is why .copy is in place)
    if "operations" in recipe:
        for level in range(1, len(recipe['objects']) + 2):
            G2 = G.copy()
            for entity, props in recipe["operations"].items():
                for entity2, props2 in G2.nodes.data():
                    if len(props2['parts_names']) < level - 1:
                        if props["consumer"] in props2["parts_names"]:
                            if props["provider"] not in props2["parts_names"]:
                                names_new = np.concatenate((props2["parts_names"], [props['provider']]))
                                types_new = np.concatenate(
                                    (props2['parts_type'], [recipe['objects'][props['provider']]['type']]))
                                for partName, partType in props2['graph'].nodes.items():
                                    if partName == props['consumer']:
                                        partNameSel = partName
                                G4 = props2['graph']
                                # TODO again .copy - was necessary not to add object to all appearances,
                                # but there should be a nicer way
                                G3 = G4.copy()
                                G3.add_node(props['provider'],
                                            part_type=recipe['objects'][props['provider']]['type'])
                                G3.add_edge(partNameSel, props['provider'])
                                G.add_node(G.number_of_nodes(), parts_names=names_new, parts_type=types_new, prob=0,
                                           graph=G3)
                                G.add_edge(entity2, G.number_of_nodes() - 1, weight=1, prob=0, action=props['type'],
                                           object=recipe['objects'][props['provider']]['type'])
                        if props["provider"] in props2["parts_names"]:
                            if props["consumer"] not in props2["parts_names"]:
                                names_new = np.concatenate((props2["parts_names"], [props['consumer']]))
                                types_new = np.concatenate(
                                    (props2['parts_type'], [recipe['objects'][props['consumer']]['type']]))
                                for partName, partType in props2['graph'].nodes.items():
                                    if partName == props['provider']:
                                        partNameSel = partName
                                G4 = props2['graph']
                                G3 = G4.copy()  # TODO ugly stuff: how to do it better that I do not overwrite
                                # also the original graph?
                                G3.add_node(props['consumer'],
                                            part_type=recipe['objects'][props['consumer']]['type'])
                                G3.add_edge(partNameSel, props['consumer'])

                                G.add_node(G.number_of_nodes(), parts_names=names_new, parts_type=types_new, prob=0,
                                           graph=G3)
                                G.add_edge(entity2, G.number_of_nodes() - 1, weight=1, prob=0, action=props['type'],
                                           object=recipe['objects'][props['consumer']]['type'])
        # nx.draw_networkx(G3, node_color='r', edge_color='b')
        # plt.draw()
        # plt.savefig('plot2')
    return G, recipe_name, assembly_name, base_filename


def prune_graph(G):
    # prunes the incoming graph G by merging the nodes, which have the same type of parts
    # incoming and outcoming edges from these nodes are merged to the similar one
    import networkx.algorithms.isomorphism as iso
    Gp = G.copy()
    # nx.draw_networkx(G, node_color='r', edge_color='b')
    # plt.draw()
    # plt.savefig('plotAll')

    # list of nodes to be removed
    remove_list = []
    # list of nodes which were considered same to the ones in remove_list
    # (to these nodes, edges from removed nodes will be added)
    similarity_list = []
    # checks each node in Gp (node1) towards each node in Gp (node2)
    # todo we want to check only when node2>node1 not to double removals, there might be some nicer way than this one...
    for entity, props in Gp.nodes.data():
        for entity2, props2 in Gp.nodes.data():
            if entity2 > entity:
                if (len(props['parts_type']) == len(props2['parts_type'])) and compare_list(props['parts_type'],
                                                                                            props2['parts_type']):
                    # nm = iso.categorical_node_match("parts_type", 'cube')
                    # em = iso.numerical_edge_match("weight", 1)
                    # if nx.is_isomorphic(props['graph'], props2['graph'], node_match=nm):
                    # TODO should check if the assembly graphs are same and isomorphic, not working so far
                    # - so far checking only number and type of the parts
                    remove_list.append(entity2)
                    similarity_list.append(entity)
        print(entity)
    x = np.array(remove_list)
    # list of unique appearance of nodes which we want to remove (in remove_list),
    # in the same way cleaning similarity list
    remove_list_u, idx_array = np.unique(x, return_index=True)
    similarity_list_u = [similarity_list[index] for index in idx_array]
    similarity_list_u = np.array(similarity_list_u)
    # remove nodes from remove_list_u and add in and out edges to the corresponding node in similarity_list_u
    for idx, rem_item in enumerate(remove_list_u):
        in_edges = Gp.in_edges(rem_item)
        out_edges = Gp.out_edges(rem_item)
        # in_edges_sim = Gp.in_edges(similarity_list_u[idx])
        # out_edges_sim = Gp.out_edges(similarity_list_u[idx])
        rem_edges = []
        for inE, outE in out_edges:
            # if the ancessor of the node (where the edge ends) is a node in the remove list,
            # find in similarity list the node to which it will be
            # merged to and exchange
            fromE = similarity_list_u[idx]
            if outE in remove_list_u:
                toE = similarity_list_u[np.where(remove_list_u == outE)][0]
            else:
                toE = outE
            # adjust the weight of the edge - increase the weight based on the number of the merged edges
            if Gp.has_edge(fromE, toE):
                weightE = nx.get_edge_attributes(Gp, 'weight')
                nx.set_edge_attributes(Gp, {(fromE, toE): {"weight": 1 + weightE[(fromE, toE)]}})
            else:
                #if the edge between the two nodes does not exist yet:
                #find by which object the two nodes differ and add edge with the corresponding parameters between fromE and toE nodes
                # objectE=list(set(Gp.nodes[toE]['parts_type']).symmetric_difference(set(Gp.nodes[fromE]['parts_type'])))
                objectEs = Counter(Gp.nodes[toE]['parts_type'])-Counter(Gp.nodes[fromE]['parts_type'])
                for a in objectEs.keys(): objectE = a
                Gp.add_edge(fromE, toE, weight=1, prob=0, object=objectE, action = None)#TODO how to find the action for the merged edges?
            rem_edges.append([inE, outE])
        for inE, outE in in_edges:
            # if the predecessor of the edge is a node in the remove list,
            # find in similarity list the node to which it will be merged to and exchange
            toE = similarity_list_u[idx]
            if inE in remove_list_u:
                fromE = similarity_list_u[np.where(remove_list_u == inE)]
            else:
                fromE = inE
            # adjust the weight of the edge - increase the weight based on the number of the merged edges
            if Gp.has_edge(fromE, toE):
                weightE = nx.get_edge_attributes(Gp, 'weight')
                nx.set_edge_attributes(Gp, {(fromE, toE): {"weight": 1 + weightE[(fromE, toE)]}})
            else:
                #if the edge between the two nodes does not exist yet:
                #find by which object the two nodes differ and add edge with the corresponding parameters between fromE and toE nodes
                # objectE=list(set(Gp.nodes[toE]['parts_type']).symmetric_difference(set(Gp.nodes[fromE]['parts_type'])))
                objectEs = Counter(Gp.nodes[toE]['parts_type'])-Counter(Gp.nodes[fromE]['parts_type'])
                for a in objectEs.keys(): objectE = a
                Gp.add_edge(fromE, toE, weight=1, prob=0, object=objectE, action = None) #TODO how to find the action for the merged edges?
            # add merged edges tot he remove list
            rem_edges.append([inE, outE])
        #remove all merged edges and then the node
        for edge in rem_edges:
            Gp.remove_edge(edge[0], edge[1])
        Gp.remove_node(rem_item)
    # nx.draw_networkx(Gp, node_color='r', edge_color='b')

    # recompute probabilities of the edges - for each node of the graph sum probabilities of the outgoing edges to 1
    with open(build_name, "r") as f:
        recipe = yaml.safe_load(f)
    # filter the nodes which have the given number of objects (same level of the assembly)
    for entity, props in Gp.nodes.data():
        out_edges = Gp.out_edges(entity)
        edges_weights = nx.get_edge_attributes(Gp, 'weight')
        sum_weights = 0
        for e in out_edges:
            sum_weights = sum_weights + edges_weights[e]
        for e in out_edges:
            nx.set_edge_attributes(Gp, {e: {'prob': round(edges_weights[e] / sum_weights, 2)}})
    # for level in range(1, len(recipe['objects']) + 2):
    #     nodes_level = [x for x, y in Gp.nodes(data=True) if len(y['parts_type']) == level]
    #     print(nodes_level)

    pos = nx.kamada_kawai_layout(Gp)
    pos = nx.circular_layout(Gp)

    pos = graphviz_layout(Gp, prog="dot")
    labels = nx.get_node_attributes(Gp, 'parts_type')
    nx.draw_networkx(Gp, pos=pos, labels=labels, font_size=6)
    label_options = {"ec": "k", "fc": "white", "alpha": 0.7}
    nx.draw_networkx_labels(Gp, pos=pos, labels=labels, font_size=6, bbox=label_options)
    labels_e = nx.get_edge_attributes(Gp, 'prob')
    nx.draw_networkx_edge_labels(G, pos=pos, edge_labels=labels_e, font_size=6, bbox=label_options)
    plt.draw()
    plt.savefig('plotPruned')
    graph_name_pickle = str.split(build_name,'.')[0]+'.txt'
    pickle.dump(Gp, open(graph_name_pickle, 'wb'))
    return Gp


def update_graph(G, Po=[], Pa=[]):
    # updates the probabilities of the nodes of the incoming graph G based on the observed probability of the observed
    # object and action
    # Po - dictionary with probability distribution over objects
    # Pa - dictionary with probability distribution over actions
    Gp = G.copy()
    # for each node:
    # node prob = sum of prob.edge*prob.observed_needed_object+action*prob.prev.state
    for n, props in Gp.nodes.data():
        prob_node = []
        in_edges = Gp.in_edges(n)
        for inE, outE in in_edges:
            print(props)
            objectE = Gp.get_edge_data(inE, outE)['object']
            probsE = Gp.get_edge_data(inE, outE)['prob']
            actionE = Gp.get_edge_data(inE, outE)['action']
            objectE_p = Po[str.lower(objectE)]
            prob_node.append(Gp.nodes[inE]['prob']*objectE_p*probsE)
        if prob_node!=[]:
            nx.set_node_attributes(G, {n: sum(prob_node)}, name='prob')
    return G

# %% Do
def compare_list(l1, l2):
    import functools
    l1.sort()
    l2.sort()
    if functools.reduce(lambda x, y: x and y, map(lambda p, q: p == q, l1, l2), True):
        return True
    else:
        return False

#TODO save after building the tree the tree and just load the saved object
graph_name_pickle = str.split(build_name,'.')[0]+'.txt'
if os.path.exists(graph_name_pickle):
    gp = pickle.load(open(graph_name_pickle, 'rb'))
    print('loading graph from a file')
else:
    print('building a new graph for the given assembly')
    g, g_name, assembly_name, base_filename = build_graph(build_name, onto)
    gp = prune_graph(g)
po = {"peg": 0.1, "cube": 0.3, "sphere": 0.2, "screw": 0.1, "other": 0.3}
pa1 = {"hammering": 0.1, "handling": 0.3, "screwing": 0.1, "other": 0.5}
pa2 = {"hammering": 0.4, "handling": 0.3, "screwing": 0.1, "other": 0.2}
gp = update_graph(gp, po, pa1)
# g = update_graph(g, po, pa2)
# %% Draw
# image_file = base_filename + ".png"
outonto_file = graph_name_pickle + ".owl"
# image = cv2.imread(image_file)
# cv2.imshow(image_file, image)
# cv2.waitKey(0)
# cv2.destroyAllWindows()
with open(f"{graph_name_pickle}_graph.txt", "w") as f:
    f.write(str(gp))

onto.serialize(outonto_file)
