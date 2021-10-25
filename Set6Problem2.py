'''Give pseudocode for an algorithm that generates an installation order for the noninstalled libraries that are needed for installing the web server library in O(P + P D)
time. Describe your algorithm. You may use any routine given in CLRS as a subroutine in your pseudocode, and you can use a textual description, a clarifying example,
or a correctness proof for the description.'''

#tomaremos las mismas clases del ejercicio anterior para vertices y grafos


def Order(graph, start):    #asumiendo las librerias vengan con sus dependencias esepcificadas
        order = []
        visited = {}
        evaluation(start, order, visited)
        return order

def evaluation(node, order, visited):
        for v in Vertex(node).get_connections():
            if v not in visited and not installed:
                visited.add(v)
                evaluation(node, order, visited)
        order.append(node)