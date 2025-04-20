import csv
import math
import heapq
import networkx
# csv from here (free) https://simplemaps.com/data/world-cities

class City:
  def __init__(self, name: str, admin_name: str, latitude: float, longitude: float, dist_from_start: int):
    self.name = name
    self.admin_name = admin_name 
    self.latitude = float(latitude)
    self.longitude = float(longitude)
    self.dist_from_start = dist_from_start

def get_all_cities(start_name, start_admin_name) -> list:
  cities = []
  start_lat = None
  start_lng = None

  with open("worldcities.csv", "r", newline="", encoding="utf-8") as csvfile:
    rows = list(csv.DictReader(csvfile))

  for row in rows:
    name = row["city"]
    admin_name = row["admin_name"]
    if name == start_name and admin_name == start_admin_name:
      start_lat = float(row["lat"])
      start_lng = float(row["lng"])
      cities.append(City(name, admin_name, start_lat, start_lng, 0))
      break

  for row in rows:
      name = row["city"]
      admin_name = row["admin_name"]

      if name == start_name and admin_name == start_admin_name:
          continue

      lat = float(row["lat"])
      lng = float(row["lng"])
      dist_from_start = haversine(lat, lng, start_lat, start_lng)
      cities.append(City(name, admin_name, lat, lng, dist_from_start))

  # Sort by distance from start
  cities.sort(key=lambda city: city.dist_from_start)
  
  return cities

# used to get distance between points (for edge weights)
# obtained from here https://www.geeksforgeeks.org/haversine-formula-to-find-distance-between-two-points-on-a-sphere/
def haversine(lat1: float, lon1: float, lat2: float, lon2: float) -> int:
  dLat = (lat2 - lat1) * math.pi / 180.0
  dLon = (lon2 - lon1) * math.pi / 180.0

  # convert to radians
  lat1 = (lat1) * math.pi / 180.0
  lat2 = (lat2) * math.pi / 180.0

  # apply formulae
  a = (pow(math.sin(dLat / 2), 2) +
        pow(math.sin(dLon / 2), 2) *
            math.cos(lat1) * math.cos(lat2))
  rad = 6371
  c = 2 * math.asin(math.sqrt(a))
  return rad * c

def get_neighbors(cities: list, base_city: City):
  '''
  given a city, returns a list of cities that are within 1000km from it
  '''
  close_cities = []

  for city in cities:
    dist = haversine(base_city.latitude, base_city.longitude, city.latitude, city.longitude)
    if dist < 1000:
      close_cities.append(city)
  return close_cities

def build_graph(cities: list, max_distance_km: int) -> networkx.Graph:
  '''
  builds the initial graph (undirected, weighted) using the networkx library
  
  only adds vertices whose edges are < 1000 (city is within 1000km from the start node)
  '''
  graph = networkx.Graph()
  cities_new = []
  for city in cities:
    if (city.dist_from_start < 1000):
      graph.add_node(city.name, city_data=city)
      # city_data is the City object
      cities_new.append(city)

  for node, data in graph.nodes(data=True):
    city = data["city_data"]
    print(f"{node}: {city.latitude}, {city.longitude}")

  for i, city1 in enumerate(cities_new):
    for j, city2 in enumerate(cities_new):
      if i < j:  # Avoid self-loops and redundant comparisons
        distance = haversine(city1.latitude, city1.longitude, city2.latitude, city2.longitude)
        if distance <= max_distance_km:
          graph.add_edge(city1.name, city2.name, weight=distance)

  return graph

def dijkstra(graph: networkx.Graph, cities: list, start_name: str, target_name: str):
    
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

        # Get neighbors dynamically based on the current city
        current_city = graph.nodes[current]['city_data']
        neighbors = get_neighbors(cities, current_city)

        for neighbor in neighbors:
          if neighbor.name in visited:
              continue

          # Lazily add neighbor to the graph if not already present
          if neighbor.name not in graph:
              graph.add_node(neighbor.name, city_data=neighbor)

          # Lazily add edge if it doesn't already exist
          if not graph.has_edge(current, neighbor.name):
              graph.add_edge(current, neighbor.name, weight=haversine(
                  current_city.latitude, current_city.longitude,
                  neighbor.latitude, neighbor.longitude))

          distance = graph[current][neighbor.name]['weight']
          new_dist = current_dist + distance

          if neighbor.name not in distances:
              distances[neighbor.name] = float('inf')
          
          if new_dist < distances[neighbor.name]:
              distances[neighbor.name] = new_dist
              previous[neighbor.name] = current
              heapq.heappush(queue, (new_dist, neighbor.name))

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

cities = get_all_cities('Gainesville', 'Florida')
graph = build_graph(cities, 10000)
path, distance = dijkstra(graph, cities, "Gainesville", "Miami")
print(f"Shortest path: {path}")
print(f"Total distance: {distance:.2f} km")