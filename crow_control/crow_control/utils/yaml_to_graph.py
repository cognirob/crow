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



class AssemblyGraphMaker():
    def __init__(self, build_name):
        self.build_name = build_name + '.yaml'
        self.graph_name = build_name + '.txt'
        self.fig_name = build_name + '.png'
        with open(self.build_name, "r") as f:
            self.recipe = yaml.safe_load(f)

    # %%Build graph
    def build_graph(self, onto, recipe_name=None, isBaseBuild=None):
        # %%Load YAML
        base_filename, _ = os.path.splitext(self.build_name)
        _, base_name = os.path.split(base_filename)


        # if os.path.exists('mydirectory/myfile.png')
        with open(self.build_name, "r") as f:
            recipe = yaml.safe_load(f)

        G = nx.DiGraph()
        # Add recipe name
        assembly_name = recipe["assembly_name"]

        Gin = nx.DiGraph()
        #Add nodes - initial node with 100% probability
        G.add_node(0, parts_names=[None], parts_type=[None], prob=1,
                   weight=1, graph=Gin)
        #Add "Other" node for false positive detections
        G.add_node(1, parts_names=['Other'], parts_type=['Other'], prob=0,
                   weight=1, graph=Gin)
        G.add_edge(0, 1, weight=1, prob=1 / (len(recipe["objects"].items()) + 1), action='None',
                   object="Other")
        #Add edge to stay in the current state
        G.add_edge(1, 1, weight=1, prob=0.55, action='None',
                   object="OtherO")
        # Add nodes - first for each object one node
        if "objects" in recipe:
            for i, (entity, props) in enumerate(recipe["objects"].items()):
                node_type = props["type"]
                Gin = nx.DiGraph()
                Gin.add_node(entity, part_type=node_type)
                G.add_node(i+2, parts_names=[entity], parts_type=[node_type], prob=0,
                           weight=1, graph=Gin)
                #Edge to connect to initial node
                G.add_edge(0, i+2, weight=1, prob=1 / (len(recipe["objects"].items()) + 1), action='move',
                           object=node_type)
                #Edge to connect to 'other' node
                G.add_edge(1, i+2, weight=1, prob=0.5, action='move',
                           object=node_type)
                #Add edge to stay in the current state
                G.add_edge(i+2, i+2, weight=1, prob=0.55, action='None',
                           object="OtherO")

        # Add nodes: in the way that for each current node it checks if there is an operation with the object in the node and if yes,
        # it adds the new part to the object
        # it is checked on individual levels - in each level, one object should be added
        # TODO improve the level thing - this was done just to not change the size of dictionary during iterations (that is why .copy is in place)
        if "operations" in recipe:
            for level in range(1, len(recipe['objects']) + 2):
                G2 = G.copy()
                for entity, props in recipe["operations"].items():
                    for entity2, props2 in G2.nodes.data():
                        # print(level)
                        # print(props2['parts_names'])
                        # if ('cube_front' in props2['parts_names']) and ('peg_neck' in props2['parts_names']):
                        #     print('hi')
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
                                    G.add_edge(entity2, G.number_of_nodes()-1 , weight=1, prob=0, action=props['type'],
                                               object=recipe['objects'][props['provider']]['type'])
                                    # Add edge to stay in the current state
                                    G.add_edge(G.number_of_nodes()-1, G.number_of_nodes()-1, weight=1, prob=0.55, action='None',
                                               object="OtherO")
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
                                    G.add_edge(entity2, G.number_of_nodes()-1 , weight=1, prob=0, action=props['type'],
                                               object=recipe['objects'][props['consumer']]['type'])
                                    # Add edge to stay in the current state
                                    G.add_edge(G.number_of_nodes()-1, G.number_of_nodes()-1, weight=1, prob=0.55, action='None',
                                               object="OtherO")
            # nx.draw_networkx(G3, node_color='r', edge_color='b')
            # plt.draw()
            # plt.savefig('plot2')
        return G, recipe_name, assembly_name, base_filename


    def prune_graph(self,G):
        # prunes the incoming graph G by merging the nodes, which have the same type of parts
        # incoming and outcoming edges from these nodes are merged to the similar one
        import networkx.algorithms.isomorphism as iso
        Gp = G.copy()

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
                    if (len(props['parts_type']) == len(props2['parts_type'])) and self.compare_list(props['parts_type'],
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
                if not inE == outE:
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
                if not inE==outE:
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
        with open(self.build_name, "r") as f:
            recipe = yaml.safe_load(f)
        # filter the nodes which have the given number of objects (same level of the assembly)
        for entity, props in Gp.nodes.data():
            out_edges = Gp.out_edges(entity)
            edges_weights = nx.get_edge_attributes(Gp, 'weight')
            sum_weights = 0
            nmb_weights = 0
            for e in out_edges:
                sum_weights = sum_weights + edges_weights[e]
                nmb_weights = nmb_weights + 1
            for e in out_edges:
                nx.set_edge_attributes(Gp, {e: {'prob': round(edges_weights[e] / sum_weights, 2)}})
            nx.set_edge_attributes(Gp, {(entity,entity): {'prob': round(((sum_weights-1)/nmb_weights)/sum_weights,2)}})
        # pos = nx.kamada_kawai_layout(Gp)
        # pos = nx.circular_layout(Gp)

        pos = graphviz_layout(Gp, prog="dot")
        labels = nx.get_node_attributes(Gp, 'parts_type')
        nx.draw_networkx(Gp, pos=pos, labels=labels, font_size=6)
        label_options = {"ec": "k", "fc": "white", "alpha": 0.7}
        nx.draw_networkx_labels(Gp, pos=pos, labels=labels, font_size=6, bbox=label_options)
        labels_e = nx.get_edge_attributes(Gp, 'prob')
        nx.draw_networkx_edge_labels(G, pos=pos, edge_labels=labels_e, font_size=6, bbox=label_options)
        plt.draw()
        plt.savefig(self.fig_name)
        # graph_name_pickle = str.split(self.build_name,'.')[0]+'.txt'
        pickle.dump(Gp, open(self.graph_name, 'wb'))
        return Gp


    def update_graph(self, G, Po=[], Pa=[]):
        # updates the probabilities of the nodes of the incoming graph G based on the observed probability of the observed
        # object and action
        # Po - dictionary with probability distribution over objects
        # Pa - dictionary with probability distribution over actions
        Gp = G.copy()
        # for each node:
        # node prob = sum of prob.edge*prob.observed_needed_object+action*prob.prev.state
        #TODO need not to recompute probability of the nonzero nodes, only lower it. How?
        pp = nx.get_node_attributes(G, 'prob')
        orig_prob = pp[0]
        # nx.set_node_attributes(G, {0: orig_prob/100}, name='prob') #make it smarter
        nx.set_node_attributes(G, {0: 0}, name='prob') #make it smarter
        print('set probability for node {} to to p = {}%'.format(0, (orig_prob/100)*100))
        prob_other = self.compute_prob_other_object(Po)

        for n, props in Gp.nodes.data():
            prob_node = []
            in_edges = Gp.in_edges(n)
            for inE, outE in in_edges:
                # print(props)
                if not inE == outE:
                    objectE = Gp.get_edge_data(inE, outE)['object']
                    probsE = Gp.get_edge_data(inE, outE)['prob']
                    actionE = Gp.get_edge_data(inE, outE)['action']
                    objectE_p = Po[str.lower(objectE)]
                    prob_node.append(Gp.nodes[inE]['prob']*objectE_p*probsE)
            if n>0:
                prob_node.append(Gp.nodes[n]['prob']*Gp.get_edge_data(n, n)['prob']*prob_other)
            if prob_node!=[]:
                nx.set_node_attributes(G, {n: sum(prob_node)}, name='prob')
                parts = nx.get_node_attributes(G, 'parts_type')
                if sum(prob_node)>0:
                    print('set probability for node {} to p = {}% ({})'.format(n,round(sum(prob_node)*100,4), parts[n]))
        return G

    def compute_prob_other_object(self, Po):
        # for the given graph G, and current observation of object probabilities Po computes what is the probability
        # that we observed other object (prob_other) than in the assembly
        # other + nothing + all the objects not in the assembly
        prob_other = 0
        object_list = []
        for item, props in self.recipe['objects'].items():
            object_list.append(str.lower(props['type']))
        for key in Po:
            if key not in object_list:
                prob_other = prob_other + Po[key]
        return prob_other

    def detect_most_probable_state(self, G):
        node_probs = nx.get_node_attributes(G, 'prob')
        max_node = max(node_probs, key = node_probs.get)
        prob = nx.get_node_attributes(G, 'prob')
        parts = nx.get_node_attributes(G, 'parts_type')
        print('max probability has a node {}: p = {}% ({}).'.format(max_node, round(prob[max_node]*100,4), parts[max_node]))
        return max_node

    def detect_next_state(self, G, max_node):
        #detect the most probable next state in graph G given the current node is max_node
        out_edges = G.out_edges(max_node)
        edge_probs = nx.get_edge_attributes(G, 'prob')
        outEs = []
        edge_prob = []
        for i, (inE, outE) in enumerate(out_edges):
            edge_prob.append(edge_probs[(inE, outE)])
            outEs.append(outE)
        next_node = outEs[edge_prob.index(max(edge_prob))]
        objects_add = nx.get_edge_attributes(G, 'object')
        object_add = objects_add[(max_node, next_node)]
        print('next most probable node is {}: p = {}% (need to add: {}).'.format(next_node, round(max(edge_prob)*100,4), object_add))
        return next_node

    def compare_list(self, l1, l2):
        import functools
        l1.sort()
        l2.sort()
        if functools.reduce(lambda x, y: x and y, map(lambda p, q: p == q, l1, l2), True):
            return True
        else:
            return False
def main():
    # %%ArgParser
    parser = argparse.ArgumentParser()
    parser.add_argument("build_path")
    parser.add_argument("build_name")
    parser.add_argument("--onto_file", "-o", default="../../../ontology/onto_draft_02.owl")
    args = parser.parse_args(["../data/","build_snake", "-o", "../../../ontology/onto_draft_02.owl"])

    # %%Initialization
    build_file = args.build_path + args.build_name
    onto_file = args.onto_file
    print(onto_file)

    # # %%Load onto
    # ONTO_IRI = "http://www.semanticweb.org/crow/ontologies/2019/6/onto_draft_01"

    onto = rdflib.Graph()
    onto.load(onto_file)
    #TODO save after building the tree the tree and just load the saved object
    # graph_name_pickle = args.build_path+ build_file+'.txt'
    am = AssemblyGraphMaker(build_file)
    if os.path.exists(am.graph_name):
        gp = pickle.load(open(am.graph_name, 'rb'))
        print('loading graph from a file')
    else:
        print('building a new graph for the given assembly')
        g, g_name, assembly_name, base_filename = am.build_graph(onto)
        gp = am.prune_graph(g)
    po = {"peg": 0.1, "cube": 0.3, "sphere": 0.2, "screw": 0.1, "other": 0.3}
    po2 = {"peg": 0.3, "cube": 0.1, "sphere": 0.1, "screw": 0.1, "other": 0.3}
    po3 = {"peg": 0.5, "cube": 0.1, "sphere": 0.1, "screw": 0.1, "other": 0.3}
    pa1 = {"hammering": 0.1, "handling": 0.3, "screwing": 0.1, "other": 0.5}
    pa2 = {"hammering": 0.4, "handling": 0.3, "screwing": 0.1, "other": 0.2}
    gp = am.update_graph(gp, po, pa1)
    gp = am.update_graph(gp, po2, pa1)
    gp = am.update_graph(gp, po3, pa1)
    max_node = am.detect_most_probable_state(gp)
    next_node = am.detect_next_state(gp, max_node)
    print()
    # outonto_file = am.build_name + ".owl"
    # with open(f"{am.graph_name}_graph.txt", "w") as f:
    #     f.write(str(gp))
    #
    # onto.serialize(outonto_file)

if __name__ == '__main__':
    main()
