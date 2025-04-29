# -*- coding: utf-8 -*-
"""
Created on Thu Jul  7 11:06:21 2022
@author: safwan
"""
import os
import random
import numpy as np
import matplotlib.pyplot as plt
import pickle
import networkx as nx
import argparse
import math


#### Notes ###
'''
The task point coordinates are specified in miles, while the distances between points are calculated in feet using the Euclidean distance formula. 

Scenarios are categorized into two types:
1. **Type A**: UAV points are identical to UGV points, representing the road network.
2. **Type B**: UAV points differ from UGV points. UGV points represent the road network, while UAV points are randomly scattered around the road network within a specified radius.

Key details:
- UAV points in Type B are generated randomly within a defined radius from UGV points.
- The coordinates of all points are generated randomly within a specified range.
- The number of points in each branch depends on the scenario scale (e.g., small, medium, or large).
- The generated scenario is saved as a pickle file in a designated folder for further use.
'''

def scenatio_generator(scale, type, plotting = True):
    """
    Generates a scenario based on the given scale and type.
    """
    

    # ----- Random point generation -----

    if scale == 'SS':
        p1 = (random.randint(1, 10), random.randint(1, 10))
        p2 = (random.randint(1, 10), random.randint(1, 10))
        p3 = (random.randint(1, 10), random.randint(1, 10))
        p4 = (random.randint(1, 10), random.randint(1, 10))

        A = np.round(np.linspace(p1, p2, 12), 2)
        B = np.round(np.linspace(p2, p3, 12), 2)
        C = np.round(np.linspace(p2, p4, 12), 2)
        
        

    if scale == 'MS':
        p1 = (random.randint(1,15),random.randint(1,15))
        p2 = (random.randint(1,15),random.randint(1,15))
        p3 = (random.randint(1,15),random.randint(1,15))
        p4 = (random.randint(1,15),random.randint(1,15))


        A = np.round(np.linspace(p1,p2,20), 2)
        B = np.round(np.linspace(p2,p3,20), 2)  
        C = np.round(np.linspace(p2,p4,20), 2)

    


    if scale == 'LS':
        p1 = (random.randint(1,24),random.randint(1,24))
        p2 = (random.randint(1,24),random.randint(1,24))
        p3 = (random.randint(1,24),random.randint(1,24))
        p4 = (random.randint(1,24),random.randint(1,24))


        A = np.round(np.linspace(p1,p2,34),2)
        B = np.round(np.linspace(p2,p3,34), 2)
        C = np.round(np.linspace(p2,p4,34), 2)
        
        


    X, Y = [], []
    for point in np.vstack((A, B, C)):
        X.append(point[0])
        Y.append(point[1])
        

    data = {}
    data['UGV_X'] = X
    data['UGV_Y'] = Y

    # in type A: uav points and ugv points are same. They are the road network.
    if type == 'A' :
          data['UAV_X'] = X
          data['UAV_Y'] = Y
        
    # in type B: uav points are different from ugv points. ugv points are the road network. uav points are scatter around the road newtork.
    if type == 'B' :

        UAV_X, UAV_Y = [], []
        if scale == 'SS':
            num_uav_points =  20 # Limit to 10 UAV points
        elif scale == 'MS':
            num_uav_points = 30  # Limit to 20 UAV points
        elif scale == 'LS':
            num_uav_points = 40  # Limit to 30 UAV points

        selected_indices = random.sample(range(len(X)), num_uav_points)
        for idx in selected_indices:
            base_x, base_y = X[idx], Y[idx]

            # Generate a random offset within a circle of radius ≤ 4 miles (uav's fuel radisus)
            radius = random.uniform(0.5, 4.0)  # at least 0.5 miles away
            angle = random.uniform(0, 2 * math.pi)

            offset_x = radius * math.cos(angle)
            offset_y = radius * math.sin(angle)

            UAV_X.append(round(base_x + offset_x, 2))
            UAV_Y.append(round(base_y + offset_y, 2))

        data['UAV_X'] = UAV_X
        data['UAV_Y'] = UAV_Y
        


    # --save the ugv points as the road network graph ---# 

    def add_edges_from_branch(branch):
        for i in range(len(branch) - 1):
            u = tuple(branch[i])
            v = tuple(branch[i+1])
            dist = np.linalg.norm(np.array(v) - np.array(u))*5280  # Convert to feet
            G.add_edge(u, v, weight=dist)


    G = nx.Graph()

    # Add edges from each branch
    add_edges_from_branch(A)
    add_edges_from_branch(B)
    add_edges_from_branch(C) 

    data['UGV_graph'] = G


    ## --save the data ---#

    output_folder = "scenarios"
    os.makedirs(output_folder, exist_ok=True)
    file_path = os.path.join(output_folder, '{}_scenario_data.pkl'.format(scale))
    with open(file_path, 'wb') as f:
        pickle.dump(data, f)






    if plotting:
        # Plot the graph
        pos = {node: node for node in G.nodes()}
        
        # Create the figure and axis
        fig, ax = plt.subplots()
        nx.draw_networkx(
    G,
    pos,
    ax=ax,
    with_labels=False,
    node_size=50,
    node_color='blue',
    edge_color='gray',
)       
        
        # Overlay UAV points (if diffferent from UGV points)
        uav_x = np.array(data['UAV_X']) 
        uav_y = np.array(data['UAV_Y'])
        plt.scatter(uav_x, uav_y, marker='o', color='red', label='UAV Points')

        uav_x = np.array(data['UGV_X']) 
        uav_y = np.array(data['UGV_Y'])
        plt.scatter(uav_x, uav_y, marker='o', color='blue', label='UGV Points')

        plt.scatter(X[0], Y[0], marker='o',  s= 150, facecolor= 'none', edgecolors= 'red', linewidths=  1.5, label='Starting depot')

        plt.xlabel("Distance")
        plt.ylabel("Distance")
        plt.xticks(fontsize=10)
        plt.yticks(fontsize=10)
        plt.axis('on')  # Ensures axis is shown
        plt.legend()
        plt.show()
    else:
        print("Plotting is disabled.")
        
        
        


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Generate UAV/UGV scenario")
    parser.add_argument("--scale", type=str, default='SS', choices=["SS", "MS", "LS"],
                        help="Scenario scale: SS (small), MS (medium), LS (large)")
    parser.add_argument("--type", type=str, default='A',  choices=["A", "B"],
                        help="Scenario type identifier (e.g., 'A')")
    parser.add_argument(
    "--plotting",
    action="store_true",
    help="Include this flag if you want to plot the scenario"
)

    args = parser.parse_args()

    scenatio_generator(scale=args.scale, type=args.type, plotting=args.plotting)

        







    
