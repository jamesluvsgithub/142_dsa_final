import csv
import math
import heapq
import networkx
# csv from here (free) https://simplemaps.com/data/world-cities

class City:
  def __init__(self, name, latitude, longitude):
    self.name = name
    self.latitude = float(latitude)
    self.longitude = float(longitude)

def get_cities():
  cities = []

  # change this to the csv you want
  with open("test.csv", newline="", encoding="utf-8") as csvfile:
    reader = csv.DictReader(csvfile)
    for row in reader:
      name = row["city"]
      lat = float(row["lat"])
      lng = float(row["lng"])
      cities.append(City(name, lat, lng))

  return cities

# used to get distance between points (for edge weights)
# obtained from here https://www.geeksforgeeks.org/haversine-formula-to-find-distance-between-two-points-on-a-sphere/
def haversine(lat1, lon1, lat2, lon2):
  dLat = (lat2 - lat1) * math.pi / 180.0
  dLon = (lon2 - lon1) * math.pi / 180.0

  # convert to radians
  lat1 = (lat1) * math.pi / 180.0
  lat2 = (lat2) * math.pi / 180.0

  # apply formulae
  a = (pow(math.sin(dLat / 2), 2) +
        pow(math.sin(dLon / 2), 2) *
            math.cos(lat1) * math.cos(lat2));
  rad = 6371
  c = 2 * math.asin(math.sqrt(a))
  return rad * c

def build_graph(cities, max_distance_km=100):
  graph = networkx.Graph()
  for city in cities:
    graph.add_node(city.name, city_data=city)

  # Add edges between cities within 100 km
  for i, city1 in enumerate(cities):
      for j, city2 in enumerate(cities):
          if i < j:  # Avoid self-loops and redundant comparisons
              distance = haversine(city1.latitude, city1.longitude, city2.latitude, city2.longitude)
              if distance <= max_distance_km:
                  graph.add_edge(city1.name, city2.name, weight=distance)
  return graph

def dijkstra(graph, start_name, target_name):
    # Priority queue: (distance, city_name)
    queue = [(0, start_name)]
    
    # Distance map
    distances = {node: float('inf') for node in graph.nodes}
    distances[start_name] = 0

    # Predecessor map (to reconstruct path)
    previous = {}

    visited = set()

    while queue:
        current_dist, current = heapq.heappop(queue)

        if current in visited:
            continue
        visited.add(current)

        if current == target_name:
            break

        for neighbor in graph.neighbors(current):
            edge_weight = graph[current][neighbor]['weight']
            new_dist = current_dist + edge_weight

            if new_dist < distances[neighbor]:
                distances[neighbor] = new_dist
                previous[neighbor] = current
                heapq.heappush(queue, (new_dist, neighbor))

    # Reconstruct shortest path
    path = []
    current = target_name
    while current in previous:
        path.append(current)
        current = previous[current]
    if current == start_name:
        path.append(start_name)
        path.reverse()
        return path, distances[target_name]
    else:
        return None, float('inf')

cities = get_cities()
graph = build_graph(cities, 100)  # Tweak this number based on density
path, distance = dijkstra(graph, "Gainesville", "Miami")
print(f"Shortest path: {path}")
print(f"Total distance: {distance:.2f} km")