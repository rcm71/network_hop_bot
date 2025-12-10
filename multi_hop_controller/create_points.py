#!/usr/bin/python3
import networkx as nx
import numpy as np
# ros hates matplot
#import matplotlib.plot as plt
from itertools import combinations


# our goal nodes. simple x,y goal (don't care for z)
# i use 1,2,3,4 as names for new nodes so use your words pls
# or shit bouta get real confusing real fast (and prob break)
class Destination():
    def __init__(self, x, y, name):
        self.x = x
        self.y = y
        self.name = name
    # end __init__()
    def __repr__(self):
        return f"Destination({self.name}, {self.x}, {self.y})"
# end Destination()


def get_destinations():
    # yo ngl wasn't tryna to write my 23049234th command line interface.
    destinations = []
    print("--- Destination Input Program ---")
    print("Enter 'done' for the destination name when finished.")
    print("-" * 35)

    while True:
        # Get the destination name
        name = input("Enter destination name: ").strip()

        # Check for the exit condition
        if name.lower() == 'done':
            print(destinations)
            return destinations
        elif not name:
            print("Name cannot be empty. Please try again. Name it 'done' if you are")
            continue

        # Get the X coordinate
        while True:
            try:
                x_input = input(f"Enter x-coordinate for {name}: ").strip()
                x = float(x_input)
                break
            except ValueError:
                print("Invalid input. Please enter a numerical value for x.")

        # Get the Y coordinate
        while True:
            try:
                y_input = input(f"Enter y-coordinate for {name}: ").strip()
                y = float(y_input)
                break
            except ValueError:
                print("Invalid input. Please enter a numerical value for y.")

        # Store the destination data
        destination = Destination(x, y, name)
        destinations.append(destination)
        print(f"'{name}' added with coordinates ({x}, {y}).")
        print("-" * 35)

    return destinations


# to have our mesh be springy, we use a force directed graph
# The thinking behind this is, given a set of points, create a
# force directed graph but when we exceed max tension (bad comms)
# we break the edge, and insert an intermediary (new drone).
# basically we run this bad boy to get where our drones should be going

# returns a dict{name, (x,y)}
def create_force_graph(destinations, max_tension, max_iter, snap_radius):
    
    # make graph
    initial_edges = []
    graph = nx.Graph()
    new_node_count = 0
    fixed_pos = {}
    for destination in destinations:
        graph.add_node(destination.name)
        fixed_pos[destination.name] = np.array([destination.x, destination.y])
        
    # make og edges
    # assumes first node is our home base, what everyone wants to connect to
    core_names = [d.name for d in destinations]
    initial_edges = []
    for i in range(1, len(core_names)):
        initial_edges.append((core_names[0], core_names[i]))
    graph.add_edges_from(initial_edges)
    
    
    # the graph moves my fixed nodes. Gonna safekeep in fixed_pos
    pos = fixed_pos.copy()
    
    for i in range(max_iter):
        print(f"\nIteration {i+1}: Nodes: {graph.number_of_nodes()}, Edges: {graph.number_of_edges()}")
        
        # spring sprong
        pos = nx.spring_layout(graph, pos=pos, iterations=42, seed=i, fixed=core_names, weight="length")
        
        # ummm thats incorrect sir
        to_break = []
        for u, v in graph.edges():
            #euclidian for now (INSERT ACTIONSERVER???)
            distance = np.linalg.norm(pos[u] - pos[v])
            if distance > max_tension:
                # GTFO
                to_break.append((u,v))
                print(f"Broke {u}<->{v} w len {distance}")
        
        # um that's actually correct sir	
        if not to_break:
            print("yippee!!")
            break

        # aw they broken
        for u,v in to_break:
            # if i were me where would i wanna be
            ideal_midpoint = (pos[u] + pos[v]) / 2.0
            
            best_snap_node = None
            min_dist_to_mid = float('inf')
            
            # if there already something around?
            for w in graph.nodes():
                # distance from the intermediary node w to the ideal midpoint
                dist_to_mid = np.linalg.norm(pos[w] - ideal_midpoint)
                
                if dist_to_mid < snap_radius and dist_to_mid < min_dist_to_mid and w != u and w != v:
                    # ohhh this is nice
                    best_snap_node = w
                    min_dist_to_mid = dist_to_mid
            
            # begone
            graph.remove_edge(u,v) 
            
            if best_snap_node:
                w = best_snap_node
                
                # Create the new edges
                graph.add_edge(v, w)
                graph.add_edge(w, u)
                
                print(f'Snapped {u}<->{v} onto existing node {w} (dist: {min_dist_to_mid:.2f})')

            else:
                # gotta make a fresh one
                new_node = f'New-{new_node_count}'
                graph.add_node(new_node)
                
                # create the new edges
                graph.add_edge(v, new_node)
                graph.add_edge(new_node, u)
                
                # plop the new node at the ideal midpoint (its initial position)
                pos[new_node] = ideal_midpoint
                
                print(f'Added new node {new_node} for {u}<->{v}')
                new_node_count += 1

    # return dict of (name, coord)
    final_positions = {}
    for node, coords in pos.items():
        final_positions[node] = coords.tolist() 
        
    return final_positions


#def draw(graph, core_names, pos):      
#    # debug visualize
#    # set logic lazines to get hard vs dynamic drones
 #   all_nodes_set = set(graph.nodes())
 #   core_nodes_set = set(core_names)
 #   hop_nodes_final = list(all_nodes_set - core_nodes_set)
 #   plt.figure(figsize=(10,8))
 ##   
  #  nx.draw_networkx_nodes(graph, pos, 
  #      nodelist=core_names, 
  #      node_color='skyblue', 
  #      label='Destination Nodes')
        
#    nx.draw_networkx_nodes(graph, pos, 
#        nodelist=hop_nodes_final, 
#        node_size=400, 
#        node_color='lightcoral', 
#        label='Intermediary Nodes')
#    nx.draw_networkx_edges(graph, pos, edge_color='gray', width=1.5)
#    nx.draw_networkx_labels(graph, pos, font_size=12, font_weight='bold')
            

    plt.title(f"Dynamic Force-Directed Graph (Final State after {i+1} iterations)")
    plt.legend(scatterpoints=1)
    plt.axis('off')
    plt.show() 
    
# end create_force_graph()
if __name__ == "__main__":
    dest = []
    
    names = ["ro", "to", "po", "ko", "lo"]
    dest = [Destination(0,0, "home"),
        Destination(2,2,"yikes"),
        Destination(10,10, "outer"),
        Destination(-15,-6, "inner"),
        Destination(-0,10, "fun"),
        Destination(-10, 10, "outthere"),
        Destination(-6,0, "pls"),
        Destination(-12,6, "plsplspls"),
        Destination(10,-10, "down")]
    
    DIST = 5.9
    create_force_graph(dest, DIST, 8, DIST)
    
# end create_force_graph()
