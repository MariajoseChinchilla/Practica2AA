#!usr/bin/env python

import sys
import os
import time
from math import *
from nhpn import *
from priority_queue import *


def distance(node1, node2):
    """Returns the distance between node1 and node2, ignoring the Earth's
    curvature.
    """
    latitude_diff = node1.latitude - node2.latitude
    longitude_diff = node1.longitude - node2.longitude
    return (latitude_diff ** 2 + longitude_diff ** 2) ** .5


def distance_curved(node1, node2):
    """Returns the distance between node1 and node2, including the Earth's
    curvature.
    """
    A = node1.latitude * pi / 10 ** 6 / 180
    B = node1.longitude * pi / 10 ** 6 / 180
    C = node2.latitude * pi / 10 ** 6 / 180
    D = node2.longitude * pi / 10 ** 6 / 180
    return acos(sin(A) * sin(C) + cos(A) * cos(C) * cos(B - D))


class NodeDistancePair(object):
    """Wraps a node and its distance representing it in the priority queue."""

    def __init__(self, node, distance):
        """Creates a NodeDistancePair to be used as a key in the priority queue.
        """
        self.node = node
        self.distance = distance

    def __lt__(self, other):
        # :nodoc: Delegate comparison to distance.
        return (self.distance < other.distance or
                (self.distance == other.distance and
                 id(self.node) < id(other.node)))

    def __le__(self, other):
        # :nodoc: Delegate comparison to distance.
        return (self.distance < other.distance or
                (self.distance == other.distance and
                 id(self.node) <= id(other.node)))

    def __gt__(self, other):
        # :nodoc: Delegate comparison to distance.
        return (self.distance > other.distance or
                (self.distance == other.distance and
                 id(self.node) > id(other.node)))

    def __ge__(self, other):
        # :nodoc: Delegate comparison to distance.
        return (self.distance > other.distance or
                (self.distance == other.distance and
                 id(self.node) >= id(other.node)))


class Network(object):
    """The National Highway Planning network."""

    def __init__(self):
        """Creates a network with nodes, links and an edge set."""
        self.nodes, self.links = self._load_data()
        self._create_adjacency_lists()
        self.edge_set = self._create_edge_set()

    def __str__(self):
        """String representation of the network size."""
        num_nodes = len(self.nodes)
        num_edges = 0
        for node in self.nodes:
            num_edges += len(node.adj)
        return "Graph size: %d nodes, %d edges" % (num_nodes, num_edges)

    def verify_path(self, path, source, destination):
        """Verifies that path is a valid path from source to destination.

        Returns:
            True if the path is valid such that it uses only edges in the edge
            set.

        Raises:
            ValueError: if the first node and the last node do not match source
                and destination respectively or if the edge in not the the edge
                set.
        """
        if source != path[0]:
            raise ValueError('First node on a path is different form the \
                              source.')
        if destination != path[-1]:
            raise ValueError('Last node on a path is different form the \
                              destination.')
        for i in range(len(path) - 1):
            if (path[i], path[i + 1]) not in self.edge_set and \
                    (path[i + 1], path[i]) not in self.edge_set:
                raise ValueError('Adjacent nodes in path have no edge between \
                                  them')
        return True

    def node_by_name(self, city, state):
        """Returns the first node that matches specified location.

        Args:
            city: the description of the node should include city.
            state: the state of the node should match state.

        Returns:
            The node if it exists, or None otherwise.
        """

        for node in self.nodes:
            if node.state == state:
                if city in node.description:
                    return node
        return None

    def _load_data(self):
        loader = Loader()
        lnodes = loader.nodes()
        llinks = loader.links()
        return lnodes, llinks

    def _create_adjacency_lists(self):
        # Use an adjacency list representation, by putting all vertices
        # adjacent to node in node.adj.
        for node in self.nodes:
            node.adj = []
        for link in self.links:
            link.begin.adj.append(link.end)
            link.end.adj.append(link.begin)

    def _create_edge_set(self):
        # Puts edges in a set for faster lookup.
        edge_set = set()
        for link in self.links:
            edge_set.add((link.begin, link.end))
        return edge_set


class PathFinder(object):
    def __init__(self, net, start, end):
        self.net = net
        self.start = start
        self.end = end

    def shortest_path(self, weight):
        start_time = time.clock()

        path, num_visited = self.dijkstra(weight, self.net.nodes,
                                          self.start, self.end)
        if path:
            if self.network.verify_path(path, self.start, self.end):
                return PathResult(path, num_visited, weight, self.net)
        else:
            return None
        
        return (p, num_visited)