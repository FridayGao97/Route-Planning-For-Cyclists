#https://towardsdatascience.com/a-star-a-search-algorithm-eb495fb156bb

#https://www.annytab.com/a-star-search-algorithm-in-python/



class Node:
    # using __slots__ for optimization
    __slots__ = ['node', 'distance', 'parent', 'osmid', 'G','g','h','f']
    # constructor for each node
    def __init__(self ,graph , osmid, distance = 0, parent = None,g=0,h=0):
        # the dictionary of each node as in networkx graph --- still needed for internal usage
        self.node = graph[osmid]
        
        # the distance from the parent node --- edge length
        self.distance = distance
        
        # the parent node
        self.parent = parent
        
        # unique identifier for each node so we don't use the dictionary returned from osmnx
        self.osmid = osmid
        
        # the graph
        self.G = graph

        #for cost f = g + h
        # g(n) — this represents the exact cost of the path from the starting node to any node n
        # h(n) — this represents the heuristic estimated cost from node n to the goal node.
        # f(n) — lowest cost in the neighboring node n
        self.g = 0
        self.h = 0
        self.f = self.g + self.h

    def get_mahanttan_dist(self, dest):
        return abs(self.G[self.osmid]['x'] - self.G[dest]['x']) + abs(self.G[self.osmid]['y'] - self.G[dest]['y'])

    # returning all the nodes adjacent to the node
    def expand(self,dest):
        children = [Node(graph = self.G, osmid = child, distance = self.node[child][0]['length'], parent = self, g = self.node[child][0]['length'], h = self.get_mahanttan_dist(dest)) \
                        for child in self.node]
        return children
    
    
    # returns the path from that node to the origin as a list and the length of that path
    def path(self):
        node = self
        path = []
        while node:
            path.append(node.osmid)
            node = node.parent
        return path[::-1]
    
    # the following two methods are for dictating how comparison works

    def __eq__(self, other):
        try:
            return self.osmid == other.osmid
        except:
            return self.osmid == other
            
    
    def __hash__(self):
        return hash(self.osmid)










# Find fittest route between each pair of POIs using A*

import time, math
from collections import deque
from tqdm import tqdm


origin_corrdinates = getOriginCorrdinates(G, ORIGIN_ADDR)

origin_node_id = ox.get_nearest_node(G, origin_corrdinates)

POIs_random = getListOfPOIs(G, number_of_pois_to_travel, origin_node_id, origin_corrdinates, search_radius, LIST_OF_POI_TAGS)
dest_node_id = POIs_random[0]

origin = Node(graph = G, osmid = origin_node_id)

destination = Node(graph = G, osmid = dest_node_id)

bar = tqdm(total = len(G))

# we will be dealing with the id of the nodes of the graph in our lists
# except for unrelaxed_node list where we need to have an actual `Node` object
# so we can invoke the path() function on it when we arrive at destination

seen = set()         # for dealing with self loops

shortest_dist = {osmid: math.inf for osmid in G.nodes()}
unrelaxed_nodes = [Node(graph = G, osmid = osmid) for osmid in G.nodes()]

shortest_dist[origin.osmid] = 0
found = False

res
open = []
closed = []
open.append(origin)

# Loop until the open list is empty
while len(open) > 0:
    # Sort the open list to get the node with the lowest cost first
    open.sort()
    # Get the node with the lowest cost
    current_node = open.pop(0)
    # Add the current node to the closed list
    closed.append(current_node)
    # Check if we have reached the goal, return the path
    if current_node == destination:
        path = []
        while current_node != origin:
            path.append(current_node.osmid)
            current_node = current_node.parent
        path.append(origin.osmid)
        # Return reversed path
        res = path[::-1]
    
    # otherwise, let's relax edges of its neighbours
    for child in current_node.expand(dest_node_id):
        if child.osmid in closed: continue
        child.g = current_node.g + child.cost
        child.h = child.get_mahanttan_dist(dest_node_id)
        child.f = child.g + child.h
        # Check if neighbor is in open list and if it has a lower f value
        if(add_to_open(open, child) == True):
            # Everything is green, add neighbor to open list
            open.append(child)
    # Return None, no path is found
print(res)    

# Check if a neighbor should be added to open list
def add_to_open(open, neighbor):
    for node in open:
        if (neighbor == node and neighbor.f > node.f):
            return False
    return True

while len(unrelaxed_nodes) > 0 and not found:
    bar.update(1); time.sleep(0.05)
    
    node = min(unrelaxed_nodes, key = lambda node : shortest_dist[node.osmid])
    
    # relaxing the node, so this node's value in shortest_dist
    # is the shortest distance between the origin and destination
    unrelaxed_nodes.remove(node)
    seen.add(node.osmid)


        
    # if the destination node has been relaxed
    # then that is the route we want
    if node == destination:
        route = node.path()
        cost = shortest_dist[node.osmid]
        found = True
        continue
    
    # otherwise, let's relax edges of its neighbours
    for child in node.expand(AVG_SPEED, paved_ways, traffic_lights):
        # skip self-loops
        if child.osmid in seen: continue
        
        # this doesn't look pretty because Node is just an object
        # so retrieving it is a bit verbose -- if you have nicer 
        # way to do that, please open an issue
        child_obj = next((node for node in unrelaxed_nodes if node.osmid == child.osmid), None)
        child_obj.cost = child.cost
        
        cost = shortest_dist[node.osmid] + child.cost
        if cost < shortest_dist[child_obj.osmid]:
            shortest_dist[child_obj.osmid] = cost
            child_obj.parent = node
            
bar.close()
print(f"The route is \n\n{route} \n\nits cost is\n\n{cost}")
















class A_star_Node:
    # using __slots__ for optimization
    __slots__ = ['node', 'distance', 'parent', 'osmid', 'G','g','h','f']
    # constructor for each node
    def __init__(self ,graph , osmid, distance = 0, parent = None,g=0,h=0):
        # the dictionary of each node as in networkx graph --- still needed for internal usage
        self.node = graph[osmid]
        
        # the distance from the parent node --- edge length
        self.distance = distance
        
        # the parent node
        self.parent = parent
        
        # unique identifier for each node so we don't use the dictionary returned from osmnx
        self.osmid = osmid
        
        # the graph
        self.G = graph

        #for cost f = g + h
        # g(n) — this represents the exact cost of the path from the starting node to any node n
        # h(n) — this represents the heuristic estimated cost from node n to the goal node.
        # f(n) — lowest cost in the neighboring node n
        self.g = 0
        self.h = 0
        self.f = self.g + self.h

    def get_mahanttan_dist(self, dest):
        return abs(self.G.nodes[self.osmid]['x'] - dest['x']) + abs(self.G.nodes[self.osmid]['y'] - dest['y'])

    # returning all the nodes adjacent to the node
    def expand(self,dest):
        children = [Node(graph = self.G, osmid = child, distance = self.node[child][0]['length'], parent = self, g = self.node[child][0]['length'], h = self.get_mahanttan_dist(dest)) \
                        for child in self.node]
        return children
    
    
    # returns the path from that node to the origin as a list and the length of that path
    def path(self):
        node = self
        path = []
        while node:
            path.append(node.osmid)
            node = node.parent
        return path[::-1]
    
    # the following two methods are for dictating how comparison works

    def __eq__(self, other):
        try:
            return self.osmid == other.osmid
        except:
            return self.osmid == other
            
    
    def __hash__(self):
        return hash(self.osmid)

    # Sort nodes
    def __lt__(self, other):
         return self.f < other.f






# Check if a neighbor should be added to open list
def add_to_open(open, neighbor):
    for node in open:
        if (neighbor == node and neighbor.f > node.f):
            return False
    return True


# Find fittest route between each pair of POIs using A*

import time, math
from collections import deque
from tqdm import tqdm


origin_corrdinates = getOriginCorrdinates(G, ORIGIN_ADDR)

origin_node_id = ox.get_nearest_node(G, origin_corrdinates)

POIs_random = getListOfPOIs(G, number_of_pois_to_travel, origin_node_id, origin_corrdinates, search_radius, LIST_OF_POI_TAGS)
dest_node_id = POIs_random[0]

origin = A_star_Node(graph = G, osmid = origin_node_id)

destination = A_star_Node(graph = G, osmid = dest_node_id)

bar = tqdm(total = len(G))

# we will be dealing with the id of the nodes of the graph in our lists
# except for unrelaxed_node list where we need to have an actual `Node` object
# so we can invoke the path() function on it when we arrive at destination

seen = set()         # for dealing with self loops

shortest_dist = {osmid: math.inf for osmid in G.nodes()}
unrelaxed_nodes = [A_star_Node(graph = G, osmid = osmid) for osmid in G.nodes()]

shortest_dist[origin.osmid] = 0
found = False


open = []
closed = []
open.append(origin)

# Loop until the open list is empty
while len(open) > 0:
    # Sort the open list to get the node with the lowest cost first
    open.sort()
    # Get the node with the lowest cost
    current_node = open.pop(0)
    # Add the current node to the closed list
    closed.append(current_node)
    # Check if we have reached the goal, return the path
    if current_node == destination:
        path = []
        while current_node != origin:
            path.append(current_node.osmid)
            current_node = current_node.parent
        path.append(origin.osmid)
        # Return reversed path
        res = path[::-1]
    
    # otherwise, let's relax edges of its neighbours
    for child in current_node.expand(G.nodes[dest_node_id]):
        if child.osmid in closed: continue
        child.g = current_node.g + child.distance
        child.h = child.get_mahanttan_dist(G.nodes[dest_node_id])
        child.f = child.g + child.h
        # Check if neighbor is in open list and if it has a lower f value
        if(add_to_open(open, child) == True):
            # Everything is green, add neighbor to open list
            open.append(child)
    # Return None, no path is found
print(res)    


