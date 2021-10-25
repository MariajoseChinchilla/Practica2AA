'''Help Alyssa by designing an algorithm that computes the strength of the strongest connection
between a given user s and every other user v to whom s is connected with vagueness at most
k'''

#recibimos de entrada un grafo con pesos
#primero hacemos la clase para los vertices y para el grafo

class Vertex:
    def __init__(self, node):
        self.id = node
        self.adjacent = {}

    def add_neighbor(self, neighbor, weight=0):
        self.adjacent[neighbor] = weight

    def get_connections(self):
        return self.adjacent.keys()

    def get_id(self):
        return self.id

    def get_weight(self, neighbor):
        return self.adjacent[neighbor]

class Graph:
    def __init__(self):
        self.vert_dict = {}
        self.num_vertices = 0

    def __iter__(self):
        return iter(self.vert_dict.values())

    def add_vertex(self, node):
        self.num_vertices += 1
        new_vertex = Vertex(node)
        self.vert_dict[node] = new_vertex
        return new_vertex

    def get_vertex(self, n):
        if n in self.vert_dict:
            return self.vert_dict[n]
        else:
            return None

    def add_edge(self, start, end, cost=0):
        if start not in self.vert_dict:
            self.add_vertex(start)
        if end not in self.vert_dict:
            self.add_vertex(end)

        self.vert_dict[start].add_neighbor(self.vert_dict[end], cost)
        self.vert_dict[end].add_neighbor(self.vert_dict[start], cost)

    def get_veretices(self):
        return self.vert_dict.keys()

    def get_edges(self):
        edges = []
        for v in self.get_veretices():
            for w in Vertex(v).get_connections():
                if (v,w) and (w,v) not in edges:
                    edges.append((v,w))
        return edges

class Solution:
    def __init__(self, graph, start, k):
        for i in range(k):
            paths = {}
            for edge in graph.get_edges():
                if edge[1] * Vertex(start).get_weight(edge[1]) > edge[0]*Vertex(start).get_weight(edge[0]) + Vertex(edge[0]).get_weight(edge[1]):
                    paths[edge[1]] = edge[0]*Vertex(start).get_weight(edge[0]) + Vertex(edge[0]).get_weight(edge[1])



    