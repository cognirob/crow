# -*- coding: utf-8 -*-
"""
Created on 1.8.2021

@author: Karla Stepanova, CIIRC CTU
"""
import rdflib
import numpy as np
import networkx as nx
import yaml
import os
import argparse
import re
import matplotlib.pyplot as plt


# %%ArgParser
parser = argparse.ArgumentParser()
parser.add_argument("build_name")
parser.add_argument("--onto_file", "-o", default="onto_draft_02.owl")
# args = parser.parse_args(["build_dog.yaml"])
args = parser.parse_args(["build_snake.yaml", "-o", "onto_draft_02.owl"])
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

    with open(build_name, "r") as f:
        recipe = yaml.safe_load(f)

    G = nx.DiGraph()
    #Add recipe name
    assembly_name = recipe["assembly_name"]

    # Add nodes - first for each object one node
    if "objects" in recipe:
        for i, (entity, props) in enumerate(recipe["objects"].items()):
            node_type = props["type"]
            Gin = nx.DiGraph()
            Gin.add_node(entity, part_type=node_type)
            G.add_node(i, parts_names=[entity], parts_type=[node_type], probs=[1 / (len(recipe) + 1)], graph=Gin)
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
                                #TODO again copy - was necessary not to add object to all appearances, but there should be a nicer way
                                G3 = G4.copy()
                                G3.add_node(props['provider'],
                                            part_type=recipe['objects'][props['provider']]['type'])
                                G3.add_edge(partNameSel, props['provider'])
                                G.add_node(G.number_of_nodes(), parts_names=names_new, parts_type=types_new, probs=[0],
                                           graph=G3)
                                G.add_edge(entity2, G.number_of_nodes() - 1, weight=0.1)
                        if props["provider"] in props2["parts_names"]:
                            if props["consumer"] not in props2["parts_names"]:
                                names_new = np.concatenate((props2["parts_names"], [props['consumer']]))
                                types_new = np.concatenate(
                                    (props2['parts_type'], [recipe['objects'][props['consumer']]['type']]))
                                for partName, partType in props2['graph'].nodes.items():
                                    if partName == props['provider']:
                                        partNameSel = partName
                                G4 = props2['graph']
                                G3 = G4.copy()  # TODO ugly stuff: how to do it better that I do not overwrite also the original graph?
                                G3.add_node(props['consumer'],
                                            part_type=recipe['objects'][props['consumer']]['type'])
                                G3.add_edge(partNameSel, props['consumer'])

                                G.add_node(G.number_of_nodes(), parts_names=names_new, parts_type=types_new, probs=[0],
                                           graph=G3)
                                G.add_edge(entity2, G.number_of_nodes() - 1, weight=0.1)
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
    #checks each node in Gp (node1) towards each node in Gp (node2)
    # todo we want to check only when node2>node1 not to double removals, there might be some nicer way than this one...
    for entity, props in Gp.nodes.data():
        for entity2, props2 in Gp.nodes.data():
            if entity2 > entity:
                if (len(props['parts_type'])==len(props2['parts_type'])) and compare_list(props['parts_type'], props2['parts_type']):
                    # nm = iso.categorical_node_match("parts_type", 'cube')
                    # em = iso.numerical_edge_match("weight", 1)
                    #if nx.is_isomorphic(props['graph'], props2['graph'], node_match=nm):
                        #TODO should check if the assembly graphs are same and isomorphic, not working so far
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
        for inE, outE in out_edges:
            # if the ancessor of the node (where the edge ends) is a node in the remove list,
            # find in similarity list the node to which it will be
            # merged to and exchange
            if outE in remove_list_u:
                Gp.add_edge(similarity_list_u[idx], similarity_list_u[np.where(remove_list_u ==outE)][0])
            else:
                Gp.add_edge(similarity_list_u[idx], outE)
        for inE, outE in in_edges:
            # if the predecessor of the edge is a node in the remove list,
            # find in similarity list the node to which it will be merged to and exchange
            if inE in remove_list_u:
                Gp.add_edge(similarity_list_u[np.where(remove_list_u == inE)], similarity_list_u[idx][0])
            else:
                Gp.add_edge(inE, similarity_list_u[idx])
        Gp.remove_node(rem_item)
    #nx.draw_networkx(Gp, node_color='r', edge_color='b')

    labels = nx.get_node_attributes(Gp, 'parts_type')
    nx.draw_networkx(Gp, pos=nx.circular_layout(Gp), labels = labels)
    plt.draw()
    plt.savefig('plotPruned')
    return Gp


# %% Do
def compare_list(l1, l2):
    import functools
    l1.sort()
    l2.sort()
    if functools.reduce(lambda x, y: x and y, map(lambda p, q: p == q, l1, l2), True):
        return True
    else:
        return False


g, g_name, assembly_name, base_filename = build_graph(build_name, onto)
gp = prune_graph(g)
# %% Draw
image_file = base_filename + ".png"
outonto_file = base_filename + ".owl"
# image = cv2.imread(image_file)
# cv2.imshow(image_file, image)
# cv2.waitKey(0)
# cv2.destroyAllWindows()
with open(f"{base_filename}_graph.txt", "w") as f:
    f.write(str(g))

onto.serialize(outonto_file)
