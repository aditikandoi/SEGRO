from __future__ import print_function
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
import numpy as np
import googlemaps
import json
import gmaps


# choose a mode
Mode = "driving"  # "driving", "walking", "bicycling", "transit"

# get Google API key from following website: 
# https://developers.google.com/maps/documentation/distance-matrix/start#get-a-key
password = "AIzaSyBM8UC4aDTqAriv05bI2mgEGaAax9Lo-sw"

# get the lat and lng of places

lat=np.array([15.3855445, 15.3929092, 15.3813679, 15.3835541, 15.3888909])
lng=np.array([73.868921 , 73.8805639, 73.8786257, 73.8707916, 73.8651697])

lat = lat.astype(float)
lng = lng.astype(float)
lat, lng


# calculate the dist_matrix
# distance unit: meter

gmaps = googlemaps.Client(key=password)

dist_matrix = []
places=len(lat)

for i in range(places):
    for j in range(places):
        x = (lat[i], lng[i])
        y = (lat[j], lng[j])
        directions_result = gmaps.directions(x,y,
                                    mode=Mode,
                                    avoid="ferries",
                                    )
        dist_matrix.append(directions_result[0]['legs'][0]['distance']['value'])
dist_matrix = np.reshape(dist_matrix, (places, places))
# dist_matrix.astype(int)
dist_matrix

# convert the dist_matrix to a symmetrical matrix

dist_matrix = np.asmatrix(dist_matrix)

for i in range(0,places, 1):
    for j in range(i+1, places, 1):
        dist_matrix[j,i] = dist_matrix[i,j]
dist_matrix = np.asarray(dist_matrix)
dist_matrix


# TSP Solver

"""Simple travelling salesman problem between cities."""

def create_data_model():
    """Stores the data for the problem."""
    data = {}
    data['distance_matrix'] = dist_matrix
    data['city_names'] = places
    data['num_vehicles'] = 1
    data['depot'] = 0
    return data


def print_solution(manager, routing, assignment):
    """Prints assignment on console."""
    print('Total distance: {} meters'.format(assignment.ObjectiveValue()))
    index = routing.Start(0)
    plan_output = 'Index:\n'
    route_distance = 0
    while not routing.IsEnd(index):
        plan_output += ' {} ->'.format(manager.IndexToNode(index))
        previous_index = index
        index = assignment.Value(routing.NextVar(index))
        route_distance += routing.GetArcCostForVehicle(previous_index, index, 0)
    plan_output += ' {}\n'.format(manager.IndexToNode(index))
    print(plan_output)
    plan_output += 'Route distance: {}miles\n'.format(route_distance)
    
    
def return_indexes(routing, assignment):
    index = routing.Start(0)
    indexes = []
    while not routing.IsEnd(index):
        previous_index = index
        index = assignment.Value(routing.NextVar(index))
        indexes = np.append(indexes, index)
    return indexes
    

def main():
    """Entry point of the program."""
    # Instantiate the data problem.
    data = create_data_model()

    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(
        len(data['distance_matrix']), data['num_vehicles'], data['depot'])

    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)


    def distance_callback(from_index, to_index):
        """Returns the distance between the two nodes."""
        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data['distance_matrix'][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

    # Solve the problem.
    assignment = routing.SolveWithParameters(search_parameters)

    # Print solution on console.
    if assignment:
        print_solution(manager, routing, assignment)
        indexes = return_indexes(routing, assignment)
    return indexes


if __name__ == '__main__':
    Index = main()

# sorting the lat and lng according to the order of tour

new_lat = [lat[0]]
new_lng = [lng[0]]

for i in range(places-1):
    index = Index[i].astype(int)
    new_lat = np.append(new_lat, lat[index])
    new_lng = np.append(new_lng, lng[index])
new_lat = np.append(new_lat, lat[0])
new_lng = np.append(new_lng, lng[0])
# new_lat = new_lat.tolist()
# new_lng = new_lng.tolist()
new_lat, new_lng



