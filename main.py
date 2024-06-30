import osmnx as ox
import networkx as nx
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
from geopy.geocoders import Nominatim
import requests
import tkinter as tk
from tkinter import filedialog, messagebox
from tkinter.scrolledtext import ScrolledText

HERE_API_KEY = 'YOUR_HERE_API_KEY'  # Replace with your HERE API key

def geocode_addresses(addresses):
    geolocator = Nominatim(user_agent="route_optimizer")
    locations = []
    for address in addresses:
        location = geolocator.geocode(address)
        if location:
            locations.append({'lat': location.latitude, 'lng': location.longitude})
        else:
            raise ValueError(f"Geocoding failed for address: {address}")
    return locations

def get_osm_distance_matrix(locations):
    G = ox.graph_from_point((locations[0]['lat'], locations[0]['lng']), dist=10000, network_type='drive')
    nodes = [ox.nearest_nodes(G, loc['lng'], loc['lat']) for loc in locations]
    distance_matrix = []
    for i, node1 in enumerate(nodes):
        row = []
        for j, node2 in enumerate(nodes):
            if i == j:
                row.append(0)
            else:
                try:
                    row.append(nx.shortest_path_length(G, node1, node2, weight='length'))
                except nx.NetworkXNoPath:
                    row.append(float('inf'))
        distance_matrix.append(row)
    return distance_matrix

def get_travel_time_with_traffic(origin, destination):
    api_url = "https://router.hereapi.com/v8/routes"
    params = {
        'transportMode': 'car',
        'origin': f"{origin['lat']},{origin['lng']}",
        'destination': f"{destination['lat']},{destination['lng']}",
        'return': 'summary',
        'apikey': HERE_API_KEY
    }
    response = requests.get(api_url, params=params)
    data = response.json()
    if 'routes' in data and len(data['routes']) > 0:
        travel_time = data['routes'][0]['sections'][0]['summary']['duration']
        return travel_time
    else:
        return float('inf')

def get_osm_traffic_time_matrix(locations):
    time_matrix = []
    for i, origin in enumerate(locations):
        row = []
        for j, destination in enumerate(locations):
            if i == j:
                row.append(0)
            else:
                travel_time = get_travel_time_with_traffic(origin, destination)
                row.append(travel_time)
        time_matrix.append(row)
    return time_matrix

def solve_tsp(time_matrix):
    manager = pywrapcp.RoutingIndexManager(len(time_matrix), 1, 0)
    routing = pywrapcp.RoutingModel(manager)

    def time_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return time_matrix[from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(time_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    solution = routing.SolveWithParameters(search_parameters)

    if solution:
        index = routing.Start(0)
        route = []
        while not routing.IsEnd(index):
            route.append(manager.IndexToNode(index))
            index = solution.Value(routing.NextVar(index))
        route.append(manager.IndexToNode(index))
        return route
    else:
        return None

def solve_tsp_nearest_neighbor(time_matrix):
    n = len(time_matrix)
    visited = [False] * n
    route = [0]
    visited[0] = True
    current_index = 0
    for _ in range(n - 1):
        next_index = min(
            (i for i in range(n) if not visited[i]),
            key=lambda i: time_matrix[current_index][i]
        )
        route.append(next_index)
        visited[next_index] = True
        current_index = next_index
    route.append(0)
    return route

def calculate_total_travel_time(time_matrix, route):
    total_time = sum(time_matrix[route[i]][route[i+1]] for i in range(len(route) - 1))
    return total_time

def optimize_route(addresses):
    locations = geocode_addresses(addresses)
    time_matrix = get_osm_traffic_time_matrix(locations)

    ortools_route_indices = solve_tsp(time_matrix)
    if ortools_route_indices is None:
        raise ValueError("Could not solve the TSP using OR-Tools.")
    ortools_route = [addresses[i] for i in ortools_route_indices]
    ortools_travel_time = calculate_total_travel_time(time_matrix, ortools_route_indices)

    nearest_neighbor_route_indices = solve_tsp_nearest_neighbor(time_matrix)
    nearest_neighbor_route = [addresses[i] for i in nearest_neighbor_route_indices]
    nearest_neighbor_travel_time = calculate_total_travel_time(time_matrix, nearest_neighbor_route_indices)

    return {
        'ortools': {'route': ortools_route, 'travel_time': ortools_travel_time},
        'nearest_neighbor': {'route': nearest_neighbor_route, 'travel_time': nearest_neighbor_travel_time}
    }

def read_addresses_from_file(file_path):
    with open(file_path, 'r') as file:
        addresses = [line.strip() for line in file if line.strip()]
    return addresses

def load_addresses():
    file_path = filedialog.askopenfilename()
    if file_path:
        addresses = read_addresses_from_file(file_path)
        addresses_text.delete('1.0', tk.END)
        addresses_text.insert(tk.END, '\n'.join(addresses))

def optimize():
    addresses = addresses_text.get('1.0', tk.END).strip().split('\n')
    if not addresses:
        messagebox.showerror("Error", "No addresses provided")
        return

    try:
        result = optimize_route(addresses)
        output_text.delete('1.0', tk.END)
        output_text.insert(tk.END, "Optimized Route (OR-Tools):\n")
        output_text.insert(tk.END, "\n".join(result['ortools']['route']))
        output_text.insert(tk.END, f"\nTotal Travel Time: {result['ortools']['travel_time']} seconds\n")
        output_text.insert(tk.END, "\nOptimized Route (Nearest Neighbor):\n")
        output_text.insert(tk.END, "\n".join(result['nearest_neighbor']['route']))
        output_text.insert(tk.END, f"\nTotal Travel Time: {result['nearest_neighbor']['travel_time']} seconds\n")
    except Exception as e:
        messagebox.showerror("Error", str(e))

# Create the main window
root = tk.Tk()
root.title("Route Optimizer")

# Create and place widgets
tk.Label(root, text="Addresses (one per line):").pack()

addresses_text = ScrolledText(root, width=40, height=10)
addresses_text.pack()

tk.Button(root, text="Load from File", command=load_addresses).pack()

tk.Button(root, text="Optimize Route", command=optimize).pack()

tk.Label(root, text="Output:").pack()

output_text = ScrolledText(root, width=40, height=10)
output_text.pack()

# Start the main event loop
root.mainloop()
