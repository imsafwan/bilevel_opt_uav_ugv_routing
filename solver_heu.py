

'''
 v2: include graph network 
 
- Heuriscs approach for performing persistent surveillance

- It is for prioritized mission points visit using proritized penalty function 

- Static recharging

- UGV route is found by solving MSC and TSP 

-  It is tried on a single agent

'''

import numpy as np
from ortools.sat.python import cp_model
import matplotlib.pyplot as plt
import pandas as pd
import time
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import time
import networkx as nx
import matplotlib.pyplot as plt
from termcolor import cprint
import os
import pickle
from matplotlib.patches import Circle


     
class Major_Replan():
    
    def __init__(self, uav_data_points,ugv_data_points,Targets,Locations,starting, metaheuristics):

        
        
        
        self.uav_data_points = uav_data_points
        self.ugv_data_points = ugv_data_points        
        self.Targets = Targets
        self.last_visit = {i : 0 for i in self.Targets }
        self.Age_period = {i : 0 for i in self.Targets }
        self.Mission_elapsed_time = 0
        self.Locations = Locations
        self.starting = starting
        self.Fuel_limit =int(287700)
        self.UAV_Mission_time = 0
        self.UAV_node_visit = [starting]
        self.UAV_node_visit_time = [0]
        self.UGV_node_visit = [starting]
        self.UGV_node_visit_time = [0]
        self.Recharging_number = 0
        self.Unvisited_Mission_points = Targets.copy()
        self.solver_time = 15
        self.metaheuristics = metaheuristics
        self.starting_depot = starting 
        self.recharging = 0
        self.ugv_speed = 15
        self.uav_speed = 33
        self.UAV_starting_point = [starting]
        self.UGV_starting_point = [starting]
        self.UGV_route_instances = []
        self.UAV_route_instances = [[],[]]
        self.UAV_fuel = [0,0]
        self.UGV_stop_location = []
        self.UGV_route_plan = []
        self.UAV_route_plan = {}
        self.recharge_map_dict = {ugv_data_points[i]: i for i in range(len(ugv_data_points))}
        
        self.map_dict = {ugv_data_points[i]: len(ugv_data_points) + i for i in range(len(ugv_data_points))}
        self.map_dict.update({uav_data_points[i]: len(ugv_data_points) + len(ugv_data_points) + i for i in range(len(uav_data_points))})
        self.action_heu = []
        self.stop_locations = []
        self.road_network_graph = None
        self.plotting = False
        
        
        

        
    
                
            
                     

    def cu_dis1 (self,node1,node2):
          '''
          returns the distance between two nodes in the ugv road network'''
          G = self.road_network_graph   
          path = nx.shortest_path(G, source=node1, target=node2, weight='weight')
          distance = int(sum(G[path[i]][path[i + 1]]['weight'] for i in range(len(path) - 1)))
          return distance               

    def path_(self,Stop1,Stop2):
        
         '''
        

        Parameters
        ----------
        Stop1 : Tuple
            Coordinate of mission point.
        Stop2 : Tuple
            Coordinate of mission point.

        Returns
        -------
        path : list of tuples
            path between two coordinates along the ugv road
        Time : list of tuples
            time taken to travel along the path.
         '''
        
         G = self.road_network_graph   
         if Stop1 == Stop2:
             path = [Stop1,Stop1]
         else:
             path = nx.shortest_path(G, source=Stop1, target = Stop2, weight='weight')
         
         path = [i for i in path if i  in self.ugv_data_points]
         
         Time = []
         t = 0
         for i in range(len(path)-1):
             t = t+ self.cu_dis1(path[i],path[i+1])/self.ugv_speed
             Time.append((int(t),600000000))
             
         return path,Time                  
                          
                      
    

   
        

    def CP_set_cover(self,Targets,Locations,starting): 
        
        '''
        modified minimum set cover algorithm
        Input : Targets = target locations, Locations = path locations , starting = starting point
        Output : Minimum number of recharging locations (with starting location) with minimum distance traversal along the path
        
        '''
         
        Targets = Targets.copy()
        A = Locations.copy() 

        def hit(point,targets):
            hitted_targets = []
            for j in range(len(targets)):
                a = np.array(point)
                b = np.array(targets[j])
                d = int(np.sqrt(np.sum(np.square(a-b)))*5280)
                if (198*d/self.uav_speed) <=(self.Fuel_limit/2):
                    hitted_targets.append(j)            
            return hitted_targets
        
        
        starting = starting
        hit_start= hit(starting,Targets)

        
        
        remv=[]
        for i in hit_start:
            remv.append(Targets[i])
        for i in remv:
            Targets.remove(i)
        
        A.remove(starting)
        
        
             
        ############## minimum hitting set #################

        model = cp_model.CpModel()
        hitted_dict = {}
        point_vars= {}
        target_set={}
        hit_set={}
        obj_var={}
        objective_set={}
        var = []
        
        for i in range(len(Targets)):  
           target_set.setdefault('Target {}'.format(i), set())  
           hit_set.setdefault('Target {}'.format(i), set()) 

        for i in range(len(A)):  
           p_var= point_vars.setdefault('Point {}'.format(i), model.NewBoolVar('Point '+str(i)))
           
           hitted_dict.setdefault('{}'.format(i), set(hit(A[i],Targets)))
           obj_var.setdefault('Point {}'.format(i),(self.cu_dis1(A[i],starting)))
        
        
        for a, b in point_vars.items():
               var.append(b)
               
        for a, b in obj_var.items():
           objective_set.setdefault(a,(b*point_vars[a]))
               
        
        for a, b in hitted_dict.items():
               for k in b:
                   hit_set['Target {}'.format(k)].add(a)

        for a, b in hit_set.items():
               for k in b:
                   target_set['{}'.format(a)].add(list(point_vars.values())[int(k)])
                    
           
        for a,b in target_set.items():
            model.Add(sum(b) >= 1)  
        #model.Minimize(sum(objective_set.values()))
        model.Minimize(sum(point_vars.values()))  
        solver = cp_model.CpSolver()
        status = solver.Solve(model)
        
        if status == cp_model.OPTIMAL:
             selected_points = [p for p in point_vars
                              if solver.Value(point_vars[p])]
             print('No of Rendzvous points: {}\n' .format((len(selected_points))+1))
        else:
             print('Unable to find an optimal solution')
             
        Locs=[]       
        for i in selected_points:
            a,b = i.split(" ")
            Locs.append(A[int(b)])
        
        
        
        task_points =    [starting] + Locs #

        def create_data_model():
    
            data = {}
            distance = []
            for i in range(len(task_points)):
                    distance.append([])
            for i in range(len(task_points)):
                    for i in range(len(task_points)):
                        for j in range(len(task_points)):
                            a = task_points[i]
                            b = task_points[j]
                            d = int(self.cu_dis1(a,b))
                            distance[i].append(d)
            data["distance_matrix"] = distance
            data["num_vehicles"] = 1
            data["depot"] = 0
            return data


        def print_solution(manager, routing, solution):
            """Prints solution on console."""
            #print(f"Objective: {solution.ObjectiveValue()} miles")
            index = routing.Start(0)
            plan_output = "Route for vehicle 0:\n"
            route_distance = 0
            route_order = []
            while not routing.IsEnd(index):
                route_order.append( task_points[manager.IndexToNode(index)])
                plan_output += f" {manager.IndexToNode(index)} ->"
                previous_index = index
                index = solution.Value(routing.NextVar(index))
                route_distance += routing.GetArcCostForVehicle(previous_index, index, 0)
            plan_output += f" {manager.IndexToNode(index)}\n"
            #print(plan_output)
            plan_output += f"Route distance: {route_distance}miles\n"

            return  route_order


        def main():
            """Entry point of the program."""
            # Instantiate the data problem.
            data = create_data_model()

            # Create the routing index manager.
            manager = pywrapcp.RoutingIndexManager(
                len(data["distance_matrix"]), data["num_vehicles"], data["depot"]
            )

            # Create Routing Model.
            routing = pywrapcp.RoutingModel(manager)


            def distance_callback(from_index, to_index):
                """Returns the distance between the two nodes."""
                # Convert from routing variable Index to distance matrix NodeIndex.
                from_node = manager.IndexToNode(from_index)
                to_node = manager.IndexToNode(to_index)
                return data["distance_matrix"][from_node][to_node]

            transit_callback_index = routing.RegisterTransitCallback(distance_callback)

            # Define cost of each arc.
            routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

            # Setting first solution heuristic.
            search_parameters = pywrapcp.DefaultRoutingSearchParameters()
            search_parameters.first_solution_strategy = (
                routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
            )
            search_parameters.time_limit.FromSeconds(self.solver_time)
            # Solve the problem.
            solution = routing.SolveWithParameters(search_parameters)

            # Print solution on console.
            if solution:
                route_order = print_solution(manager, routing, solution)

            return route_order

        Locs = main()
        Locs =  [starting] + list(dict.fromkeys(Locs))

       

        self.stop_locations = Locs.copy()
        
        return Locs


                      
                              
    def Allocation_function(self,Targets,Locations,starting):
        '''
        Allocation function for UAV mission points to UGV recharging locations
        '''
        Allocation = {}
        Targets = Targets
        Locations = Locations
        starting = starting
        
        
        Locations = Locations
        Refuel_stop_Locs = self.stop_locations.copy()
        
        Targets = Targets
        
        
        
        # UGV path determination #
        UGV_path = []
        
        if len(Refuel_stop_Locs) == 1: # if only one recharging location
            Refuel_stop_Locs = [Refuel_stop_Locs[0],Refuel_stop_Locs[0]]
        
        i = 1
        while i < len(Refuel_stop_Locs): # creating the path
              UGV_path_i,UGV_time = self.path_(Refuel_stop_Locs[i-1],Refuel_stop_Locs[i])
              UGV_path = UGV_path + UGV_path_i
              i = i+1
            
            
        
        Locations_copy =  [x for x in Targets if x not in UGV_path]  # Target for UAV , not in UGV path
        
        
        Allc = {}

        for i in Refuel_stop_Locs:
              Allc.setdefault((i),[])

        for i in Locations_copy :
              Allocated = []
              for j in Refuel_stop_Locs:
                  a = np.array(i)
                  b = np.array(j)
                  d = int(np.sqrt(np.sum(np.square(a-b)))*5280)
                  if d*6 <= self.Fuel_limit/2:
                      Allocated.append(j)    # a list that shows uav points and its covering rf stops
              Allocated.sort(key=lambda x:int(np.sqrt(np.sum(np.square(np.array(x)-np.array(i))))*5280))
              
              
              Allc[(Allocated[0])].append(i) # add that uav points to the nearest rf stop
        
        Allocation = {}
        for j in range(len(Refuel_stop_Locs)-1):
            
            if j == 0 : 
                    Allocation[starting, Refuel_stop_Locs[j+1]] = Allc[starting] + Allc[Refuel_stop_Locs[j+1]]
            else : 
                    Allocation[Refuel_stop_Locs[j], Refuel_stop_Locs[j+1]] = Allc[Refuel_stop_Locs[j+1]]
        

        cprint('\n Allocation of Mission points obtained = \n {} \n \n'.format( Allocation ), 'cyan', attrs = ['bold'] ) 
        
          
        
        if self.plotting:
            
            ##### plots of the task allocation #####
            fig, axes = plt.subplots(1, len(Allocation), figsize=(15, 5), constrained_layout=True)
            
            if len(Allocation) == 1:
                axes = [axes]
            rf = 0
            
            for ax, (key, values) in zip(axes, Allocation.items()):
                ax.scatter(self.starting[0],self.starting[1],s=80,color = 'red',marker = 'o')
                for i in self.Targets:
                    ax.scatter(i[0],i[1],s=80,color = 'black',marker = 'x')
                fuel_radius = ((self.Fuel_limit * self.uav_speed) / (198*2))  # in ft
                circle = Circle((key[1][0], key[1][1]), radius=fuel_radius * 0.000189394, color='b', alpha=0.15, ec=None)
                ax.add_patch(circle)
                ax.scatter(key[1][0], key[1][1], s=160, color='b', marker='.', label='Refuel Stop {}'.format(rf))
                ax.set_xlim(0, 15)
                ax.set_ylim(0, 15)
                ax.set_aspect('equal', adjustable='box')
                ax.legend()
                rf += 1
                fig.suptitle('UGV path and UAV mission points allocation')  # Use suptitle for the figure

        
            plt.show()
        return Allocation
    
    
    
    
    
    
    
    


    
    
        
    def UAV_planner(self, starting_location, Mission_points, UGV_end_stop, UGV_end_stop_time):
        
    
              '''
              

              Parameters
              ----------
              MP : list of tuples
                  mission points.
              UGV_points : list of tuples
                  ugv path.
              UGV_time : list
                  ugv time.

              Returns
              -------
              TYPE
                  route.

              '''
              
              def create_data_model(print_solution= True):
                  
                  """Stores the data for the problem."""
                  
                  data = {}
                  distance = []
                  N_d_r =  6   #No_of_duplicate_nodes_for_recharging
                  
                  # as a form of coordinates #
                  A_s = [starting_location]
                  A_v = Mission_points
                  A_r =  [UGV_end_stop]*N_d_r
                  A_e = [UGV_end_stop]
                  
                  # as a form of nodes #
                  starting_node = [0]
                  mission_point_node = list(range(len(starting_node),len(A_s +A_v)))
                  refuel_stop_nodes =  list(range(len(A_s+A_v), len(A_s+A_v+A_r)))
                  end_node = [len(A_s+ A_v+ A_r)]
                  
                  data['starting node'] = starting_node
                  data['mission_point_node'] = mission_point_node
                  data['refuel_stop_nodes'] = refuel_stop_nodes
                  data['end_node'] = end_node
                  
                  
                  

                  
                          
                  #creation of distance matrix#
                  A1 = A_s + A_v + A_r + A_e # all coordinates
                  
                  for i in range(len(A1)):
                     distance.append([])
                  for i in range(len(A1)):
                      for j in range(len(A1)):
                          a = A1[i]
                          b = A1[j]
                          a = np.array(a)
                          b = np.array(b)
                          #if i in refuel_stop_nodes and j in refuel_stop_nodes :
                              #d = 10000
                          #else: 
                          d = int(np.sqrt(np.sum(np.square(a-b)))*5280)
                          distance[i].append(d)
                  
                  
                  
                  

                  data['distance_matrix'] = distance
                  data['num_vehicles'] = 1
                  data['starts'] = starting_node 
                  data['locations']= A1
                  data['ends'] = [len(A1)-1]*data['num_vehicles']
                  
                  data['refuel_stations'] = list(range(len(A_s+A_v), len(A_s+A_v+A_r)))
                  data['visit_stations'] = list(range(1,len(A_s +A_v)))
                  tws = [(0,60000000)]*data['num_vehicles']
                  tw1 = [(0,600000000)]*len(A_v)
                  tw2 =  [(UGV_end_stop_time, 60000000)]*N_d_r
                  tw_e = [(UGV_end_stop_time, 60000000)]
                  

                  data['time_windows'] = tws+tw1+tw2+tw_e
                  data['fuel_limit'] = self.Fuel_limit
                  data['vehicle_speed']= self.uav_speed
                  data['visit_end_nodes'] = len(A_s+A_v)-1 
                  data['station_end_nodes'] = len(A1)-2  
                  

                  
                  print('Time window', data['time_windows'])
                  #print('Starts',data['starts'] , 'Ends', data['ends'])
                  

                  assert len(data['distance_matrix']) == len(data['time_windows'])
                  assert len(data['starts']) == len(data['ends'])
                  assert data['num_vehicles'] == len(data['starts'])
                  
                  #print('Refuel stops ', A_r)
                  data['Refuel_locs'] = A_r
                  data['Refuel_time']= tw2
                  data['Mission_locs'] = A_v
                  
                  
                  
                  
                  
                  
                  return data


              def print_solution(data, manager, routing, solution): 
                  
                      data = create_data_model(print_solution= False)
                      locations = data['locations']
                       
                      
                      #print("Objective: {}".format(solution.ObjectiveValue()))  
                      obj =  solution.ObjectiveValue()
                      total_fuel = 0
                      total_time = 0
                      total_distance = 0
                      
                      distance_dimension = routing.GetDimensionOrDie("Distance")
                      fuel_dimension = routing.GetDimensionOrDie("Fuel")
                      time_dimension = routing.GetDimensionOrDie("Time")
                      # Display dropped nodes.
                      dropped_nodes = 'Dropped nodes:'
                      dropped_locs = []  
                      for node in range(routing.Size()):
                         if routing.IsStart(node) or routing.IsEnd(node):
                             continue
                         if solution.Value(routing.ActiveVar(node)) == 0:
                             dropped_nodes += ' {} node {}'.format(manager.IndexToNode(node),locations[manager.IndexToNode(node)])
                             #if locations[manager.IndexToNode(node)] in A_v:
                             if node in data['mission_point_node']:
                                  dropped_locs.append(locations[manager.IndexToNode(node)])
                             
                            
                      print('Dropped in SP ={}'.format(dropped_locs) )      
                      
                      
                      
                      
                      
                      
                      for vehicle_id in range(data['num_vehicles']):
                          
                          print('UAV id >>>>>>>>>>>>>>> ', vehicle_id)
                          index = routing.Start(vehicle_id)
                          plan_output = 'Route for vehicle {}:\n'.format(vehicle_id)
                          
                          
                          ugv_starting_location = starting_location
                         
                          Travelled_node = []
                          Travelled_time_start = []
                          Travelled_time_end = []
                          waiting_time_UAV = []           
                          Fuel_UAV = []
                          
                          
                          
                          while not routing.IsEnd(index):
                              dist_var = distance_dimension.CumulVar(index)
                              fuel_var = fuel_dimension.CumulVar(index)
                              time_var = time_dimension.CumulVar(index)
                              slack_var = time_dimension.SlackVar(index)
                        
                              Travelled_node.append(locations[manager.IndexToNode(index)])
                              
                                 
                                       
                              Travelled_time_start.append(solution.Min(time_var))
                              Travelled_time_end.append(solution.Max(time_var))
                              waiting_time_UAV.append(solution.Max(slack_var))
                              Fuel_UAV.append(solution.Value(fuel_var))
                              

                                  
                              plan_output += "{0} Node {1} Fuel({2}) Time({3},{4}) TimeSlack ({5}) Distance({6}) -> ".format(
                                  manager.IndexToNode(index),locations[manager.IndexToNode(index)],
                                  solution.Value(fuel_var),
                                  solution.Min(time_var), solution.Max(time_var),solution.Value(slack_var), solution.Value(dist_var))

                              previous_index = index
                              index = solution.Value(routing.NextVar(index))  
                              
                               
                              if locations[manager.IndexToNode(index)] == UGV_end_stop:
                                  
                                  if locations[manager.IndexToNode(index)] != locations[manager.IndexToNode(previous_index)] :
                                      
                                  
                                     self.action_heu.append((self.recharge_map_dict[locations[manager.IndexToNode(index)]], 1))
                              
                                     UGV_travel_path, UGV_travel_time = self.path_(ugv_starting_location, UGV_end_stop)
                              
                                     for i in UGV_travel_path :
                                          self.action_heu.append((self.map_dict[i], 0))
                              
                                     self.action_heu.append((self.recharge_map_dict[i], 0))
                                     ugv_starting_location = UGV_travel_path[-1]
                              
                              else:
                                     self.action_heu.append((self.map_dict[locations[manager.IndexToNode(index)]], 1))

                                     
                          
                          dist_var = distance_dimension.CumulVar(index)
                          fuel_var = fuel_dimension.CumulVar(index)
                          time_var = time_dimension.CumulVar(index)
                          
                          
                      
                          Travelled_node.append(locations[manager.IndexToNode(index)])
                          Travelled_time_start.append(solution.Min(time_var))
                          Travelled_time_end.append(solution.Max(time_var))
                          waiting_time_UAV.append(solution.Max(slack_var))
                          Fuel_UAV.append(solution.Value(fuel_var))
     
                          
                              
                          plan_output += "{0} Node {1} Fuel({2}) Time({3},{4}) TimeSlack ({5}) Distance({6}) -> ".format(
                              manager.IndexToNode(index),locations[manager.IndexToNode(index)],
                              solution.Value(fuel_var),
                              solution.Min(time_var), solution.Max(time_var),solution.Value(slack_var), solution.Value(dist_var))
                          plan_output += "Distance of the route: {} ft\n".format(solution.Value(dist_var))
                          plan_output += "Remaining Fuel of the route: {}\n".format(solution.Value(fuel_var))
                          plan_output += "Total Time of the route: {} seconds\n".format(solution.Value(time_var))
                          cprint('UAV sortie --->', 'red', attrs = ['bold'])
                          print(plan_output)
                          total_distance += solution.Value(dist_var)
                          total_fuel += solution.Value(fuel_var)
                          total_time += solution.Value(time_var)

                          sortie_time = total_time
                      
                          
                          
                              
                          
                          
                          return sortie_time, dropped_locs
                      
                      
            
                      
                      
                      
                      
                      
                      



              def main():
                  
                  """Solve the E-VRPTW problem."""
                  
                  # Instantiate the data problem.

                  data = create_data_model() 
                  refuel_stations = data['refuel_stations']
                  fuel_limit = data['fuel_limit']
                  mission_point_node = data['mission_point_node']
                  refuel_stop_nodes = data['refuel_stop_nodes']

                  
                
                  # Create the routing index manager.
                  manager = pywrapcp.RoutingIndexManager(len(data['distance_matrix']),
                                                         data['num_vehicles'],data['starts'],
                                                         data['ends'])

                  # Create Routing Model.
                  routing = pywrapcp.RoutingModel(manager)

                  solver = routing.solver()
                  
                  
                  
                  #------------------------------------------ distance dimension ---------------------------------------------------#

                  
                  def distance_callback(from_index, to_index):
                      """Returns the distance between the two nodes."""
                      # Convert from routing variable Index to distance matrix NodeIndex.
                      from_node = manager.IndexToNode(from_index)
                      to_node = manager.IndexToNode(to_index)
                      return data['distance_matrix'][from_node][to_node]

                  transit_callback_index = routing.RegisterTransitCallback(distance_callback)

                  # Add Distance constraint.
                  dimension_name = 'Distance'

                  routing.AddDimension(
                      transit_callback_index,
                      0,  # no slack
                      300000000000,  # vehicle maximum travel distance
                      True,  # start cumul to zero
                      dimension_name)
                  
                  distance_dimension = routing.GetDimensionOrDie(dimension_name)
                  
                  #-------------------------------------------- time dimension ------------------------------------------------------#

                  
                  def time_callback(from_index, to_index):
                      from_node = manager.IndexToNode(from_index)
                      to_node = manager.IndexToNode(to_index)
                      if to_node in refuel_stations:  # or to_node in end_node:                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 
                          refuel_time = 900
                      else:
                         refuel_time = 0
                      t = int(data["distance_matrix"][from_node][to_node] / data["vehicle_speed"]) + refuel_time
                      return t


                  time_callback_index = routing.RegisterTransitCallback(time_callback)
                  routing.SetArcCostEvaluatorOfAllVehicles(time_callback_index)

                  
                  routing.AddDimension(
                      time_callback_index,
                      1000,  # max slack/wait time
                      10000000000000,  # vehicle max time
                      False,  # don't force cumul to zero
                      'Time')
                  
                  time_dimension = routing.GetDimensionOrDie('Time')

                  
                  makespan = solver.IntVar(0, 10000, 'makespan')
                  # Constrain makespan to be greater than or equal to each vehicle's end time
                  for vehicle_id in range(data['num_vehicles']):
                        end_index = routing.End(vehicle_id)
                        end_time = time_dimension.CumulVar(end_index)
                        solver.Add(makespan >= end_time)
                  routing.AddVariableMinimizedByFinalizer(makespan)
                  
                  
                  for location_idx, time_window in enumerate(data["time_windows"]):
                      if location_idx in data["starts"] or location_idx in data["ends"]:
                          continue
                      index = manager.NodeToIndex(location_idx)
                      routing.AddToAssignment(time_dimension.SlackVar(index))
                      time_dimension.SlackVar(index).SetValue(0)
                      time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])
                      routing.AddVariableMinimizedByFinalizer(time_dimension.CumulVar(index))


                  for vehicle_id in range(data["num_vehicles"]):
                      index = routing.Start(vehicle_id)
                      #time_dimension.SlackVar(index).SetValue(0)
                      time_dimension.CumulVar(index).SetRange(data["time_windows"][0][0],
                                                              data["time_windows"][0][1])                      
                      routing.AddToAssignment(time_dimension.SlackVar(index))

                  for vehicle_id in range(data["num_vehicles"]):
                      end_index = routing.End(vehicle_id)
                      depot_end_idx = data["ends"][vehicle_id]  # Map to the correct end depot
                      time_dimension.CumulVar(end_index).SetRange(
                        data["time_windows"][depot_end_idx][0],
                        data["time_windows"][depot_end_idx][1]
                      )
                      time_dimension.SetCumulVarSoftUpperBound(end_index, 0, 10000)
                  
                    
 
                  # Iterate over all vehicles
                  for vehicle_id in range(data["num_vehicles"]):
                      end_index = routing.End(vehicle_id)  # Get the end index for the vehicle
                      conditions = []   
                      for refuel_stop in refuel_stop_nodes :
                          index0 = manager.NodeToIndex(refuel_stop)
                          condition = (routing.NextVar(index0) == end_index)
                          conditions.append(condition)
                      solver.Add(solver.Sum(conditions) == 1)


                  
                  for i in range(manager.GetNumberOfVehicles()): 
                      routing.AddVariableMinimizedByFinalizer(
                          time_dimension.CumulVar(routing.Start(i)))
                      routing.AddVariableMinimizedByFinalizer(
                          time_dimension.CumulVar(routing.End(i)))
                      
                      
                  #--------------------------------------------- fuel constraints -----------------------------------------------------#
                 
                  
                  def fuel_callback(from_index,to_index):
                      from_node = manager.IndexToNode(from_index)
                      to_node = manager.IndexToNode(to_index)
                      
                      
                          
                      if to_node in refuel_stations :
                          refuel = - fuel_limit
                      else:
                          refuel = 0
                      return ( refuel+ 198*int(data["distance_matrix"][from_node][to_node] / data['vehicle_speed']))
                  

                  fuel_callback_index = routing.RegisterTransitCallback(fuel_callback)
                  routing.AddDimension(
                      fuel_callback_index,
                      data['fuel_limit'],
                      data['fuel_limit'],
                      False,
                      'Fuel')

                  fuel_dimension = routing.GetDimensionOrDie('Fuel')
                  
                  
                  
                   

                  
                  for node in refuel_stop_nodes: 
                      idx = manager.NodeToIndex(node) 
                      #fuel_dimension.SlackVar(idx).SetValue(0)
                      fuel_dimension.CumulVar(idx).SetValue(0)
                      
                  for node in mission_point_node:
                      idx = manager.NodeToIndex(node) 
                      fuel_dimension.SlackVar(idx).SetValue(0)
                  
                    
                  
                  
                  # ------------------------------------ penalty ------------------------------------------------------- #
                  
                  for node in refuel_stop_nodes:
                      
                      routing.AddDisjunction([manager.NodeToIndex(node)], 0)
                      
                  penalty = 1000000000
                  for i in range(len(mission_point_node)):
                      
                      node =  mission_point_node[i]
                      routing.AddDisjunction([manager.NodeToIndex(node)], penalty)                 
                  
                 
                  

                  #------------------------------------ solver -----------------------------------------------------------#

                  search_parameters = pywrapcp.DefaultRoutingSearchParameters()
                  search_parameters.first_solution_strategy = (
                      routing_enums_pb2.FirstSolutionStrategy.PATH_MOST_CONSTRAINED_ARC)
                  
                  
                  if self.metaheuristics == "TABU_SEARCH":
                       meta_heuristics_enum = routing_enums_pb2.LocalSearchMetaheuristic.TABU_SEARCH
                  elif self.metaheuristics == "SIMULATED_ANNEALING":
                       meta_heuristics_enum = routing_enums_pb2.LocalSearchMetaheuristic.SIMULATED_ANNEALING
                  elif self.metaheuristics == "GUIDED_LOCAL_SEARCH":
                       meta_heuristics_enum = routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
                  search_parameters.local_search_metaheuristic = (meta_heuristics_enum)
                  
                  
                  search_parameters.time_limit.FromSeconds(self.solver_time)


                  # Solve the problem.
                  solution = routing.SolveWithParameters(search_parameters)

                  

                  # Print solution on console.
                  if solution:
                      dropped_locs = print_solution(data, manager, routing, solution)
                      
                  else:
                       print('no solution')   
                  
                    
                  return dropped_locs
              
                
              #if __name__ == '__main__':
              sortie_time, dropped_locs = main()
             
               
              return sortie_time, dropped_locs
            
      
    
      
        
        
               
            
    

#-----------------------    Framework    -------------------------#

def run_scenario_with_metaheuristic(metaheuristic, folder_name, plotting, scale, solver_time):

    
    
    

    if not os.path.exists(folder_name):
       os.makedirs(folder_name) 

    start_time = time.time()
    
    cprint('\n <-----------------------Solving one scenario -----------------------------------> \n', 'yellow', attrs = ['bold'])
    
    file_path = os.path.join(folder_name, '{}_scenario_data.pkl'.format(scale))
    with open(file_path , 'rb') as f:
        data = pickle.load(f)
    
    
    #df = pd.read_csv((os.path.join(folder_name, 'scenario_SS.csv')))
    uav_points_x = data['UAV_X']
    uav_points_y = data['UAV_Y']

    ugv_points_x = data['UGV_X']
    ugv_points_y = data['UGV_Y']

    uav_data_points= [(round(x,2), round(y,2)) for x, y in zip(uav_points_x, uav_points_y)]
    ugv_data_points= [(round(x,2), round(y,2)) for x, y in zip(ugv_points_x, ugv_points_y)]
    depot = [0]

    Targets = list(dict.fromkeys(uav_data_points + ugv_data_points))
    Locations = ugv_data_points
    starting  = ugv_data_points[depot[0]] 
    

    replan = Major_Replan(uav_data_points, ugv_data_points, Targets, Locations, starting, metaheuristic)
    replan.solver_time = solver_time
    replan.road_network_graph = data['UGV_graph']
    replan.plotting = plotting
    
    
    if plotting:

        ### plotting the scenario ###
        fig, ax = plt.subplots() 
        ax.scatter(starting[0],starting[1],s=80,color = 'red',marker = 'o', label='Starting Depot')
        ax.scatter(np.array(Targets)[:, 0], np.array(Targets)[:, 1], s=80, color='black', marker='x', label='Mission Points')
        ax.set_xlim(0,15)
        ax.set_ylim(0,15)
        ax.set_aspect('equal', adjustable='box')
        plt.title('Scenario')   
        plt.legend()
        plt.show()
    

    mission_points_unvisited  =   replan.Unvisited_Mission_points 
    road_network = Locations
    UGV_location = replan.UGV_starting_point[0]
    
    
    
    '''----- minimum set cover algorithm (MSC) ------'''
    stop_locs_list = replan.CP_set_cover(mission_points_unvisited, road_network, UGV_location)     
    
    
    if len(stop_locs_list) == 1:    # when there is only 1  refuel stop
        stop_locs_list = stop_locs_list + stop_locs_list
    
    '''----- Allocation ------'''

    Allocation = replan.Allocation_function(mission_points_unvisited, road_network, UGV_location)
    
    
    ''' ---- Loop until all points are covered ----'''
    
    
    Total_mission_time = 0
    subproblem = 1

    for key, value in Allocation.items() :
        print(' ')
        print('----------------------------------------------------------------------------------------------')
        print('\n solving subproblem {} >>>>>> \n'.format(subproblem))
        subproblem += 1
        

        UGV_stops =  key
        UAV_points = value
        
        
        UGV_start_stop = UGV_stops[0]
        UGV_end_stop = UGV_stops[1]
        
        UGV_end_stop_time = int(replan.cu_dis1(UGV_start_stop, UGV_end_stop)/replan.ugv_speed)

        
        if plotting:

            ### plotting subproblems ###
            fig, ax = plt.subplots()
            ax.scatter(starting[0],starting[1],s=120,color = 'red',marker = 'o', label='Starting Depot')
            ax.scatter(np.array(Targets)[:, 0], np.array(Targets)[:, 1], s=80, color='black', marker='x', label='Mission Points')
            ax.set_xlim(0,15)
            ax.set_ylim(0,15)
            ax.set_aspect('equal', adjustable='box')
            ax.scatter(UGV_start_stop[0], UGV_start_stop[1], s=80, color='blue', marker='o', label='UGV Start')
            ax.scatter(UGV_end_stop[0], UGV_end_stop[1], s=80, color='orange', marker='o', label='UGV End')
            if len(UAV_points) > 0:
                ax.scatter(np.array(UAV_points)[:, 0], np.array(UAV_points)[:, 1], s=80, color='r', marker='x', label='UAV Points')
            ax.set_title('Subproblem {}'.format(subproblem-1))
            plt.legend()
            plt.show()
        
        
        
        Task_points_of_SP = UAV_points
        starting_location = UGV_stops[0]   # starting location of the UAV 
        
        
        
            
        dropped_locs =  Task_points_of_SP
        if dropped_locs == []:
            cprint('All points covered by UGV', 'green', attrs = ['bold'])
            print('UGV travel time = {} min'.format(round(UGV_end_stop_time/60), 2))
            Total_mission_time += UGV_end_stop_time
            continue

        No_of_times = 0  # in any case if it enters in infinite loop, it will break after 15 iterations
        while len(dropped_locs) != 0 : 
            print('solving uav sorties --->')
            sortie_time, dropped_locs = replan.UAV_planner(starting_location, Task_points_of_SP, UGV_end_stop, UGV_end_stop_time)
            print('UAV sortie time = {} min'.format(round(sortie_time/60), 2))
            Task_points_of_SP = dropped_locs
            starting_location = UGV_end_stop
            UGV_end_stop_time = 0
            No_of_times = No_of_times + 1 
            Total_mission_time += sortie_time
            if No_of_times == 15:
                print('Infinite loop in solving a subproblem')
                break
    

    print('----------------------------------------------------------------------------------------------')
    print('\n UGV returning to starting depot--->')
    ugv_return_time = replan.cu_dis1(UGV_end_stop, starting)/replan.ugv_speed
    Total_mission_time += ugv_return_time
    print('UGV return travel time = {} min'.format(round(ugv_return_time/60), 2))

    end_time = time.time() 
    total_time = end_time - start_time

    
    cprint('\n Total mission time = {} minutes \n'.format(round(Total_mission_time/60), 2), 'cyan', attrs = ['bold'])
     
    
          
            
            
import argparse

def main_solver(metaheuristics, folder_name, plotting, scale, solver_time):
     run_scenario_with_metaheuristic(metaheuristics, folder_name, plotting, scale, solver_time)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run scenario solver with metaheuristics")

    parser.add_argument("--scale", type=str, default='SS', choices=["SS", "MS", "LS"],
                        help="Scenario scale: SS (small), MS (medium), LS (large)")
    
    parser.add_argument("--methods", type=str, default='GUIDED_LOCAL_SEARCH',
                        help="List of metaheuristic methods to run. Example: --methods TABU_SEARCH")
    parser.add_argument("--solver_time", type=int, default=15,
                        help="Solver time for solving E-VRPTW inside UAV planner")
    parser.add_argument("--folder", type=str, default='scenarios',
                        help="Folder name to store/load scenarios")
    
    parser.add_argument(
        "--plotting",
        type=lambda x: (str(x).lower() == 'true'),
        default=True,
        help="Set plotting True or False"
    )
    args = parser.parse_args()
    
    

    main_solver(metaheuristics=args.methods, folder_name=args.folder, plotting=args.plotting, scale=args.scale, solver_time=args.solver_time)   
    
    
    
    
