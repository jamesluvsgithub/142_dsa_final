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

def get_all_cities(start_name: str, start_admin_name: str, file_name: str) -> list:
  cities = []
  start_lat = None
  start_lng = None

  with open(file_name, 'r', newline='', encoding='utf-8') as csvfile:
    rows = list(csv.DictReader(csvfile))

  for row in rows:
    name = row['city']
    admin_name = row['admin_name']
    if name == start_name and admin_name == start_admin_name:
      start_lat = float(row['lat'])
      start_lng = float(row['lng'])
      cities.append(City(name, admin_name, start_lat, start_lng, 0))
      break

  for row in rows:
      name = row['city']
      admin_name = row['admin_name']

      if name == start_name and admin_name == start_admin_name:
          continue

      lat = float(row['lat'])
      lng = float(row['lng'])
      dist_from_start = haversine(lat, lng, start_lat, start_lng)
      cities.append(City(name, admin_name, lat, lng, dist_from_start))

  cities.sort(key=lambda city: city.dist_from_start)
  
  return cities

def haversine(lat1: float, lon1: float, lat2: float, lon2: float) -> int:
  '''
  used to get distance between points (for edge weights)
  obtained from here https://www.geeksforgeeks.org/haversine-formula-to-find-distance-between-two-points-on-a-sphere/
  '''
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

  for i, city1 in enumerate(cities_new):
    for j, city2 in enumerate(cities_new):
      if i < j:  # avoid self loops and redundant comparisons
        distance = haversine(city1.latitude, city1.longitude, city2.latitude, city2.longitude)
        if distance <= max_distance_km:
          graph.add_edge(city1.name, city2.name, weight=distance)

  return graph

def dijkstra(graph: networkx.Graph, cities: list, start_name: str, target_name: str):
    queue = [(0, start_name)]
    
    # distance map
    distances = {node: float('inf') for node in graph.nodes}
    distances[start_name] = 0

    # predecessor map (to reconstruct path)
    previous = {}
    visited = set()

    while queue:
        current_dist, current = heapq.heappop(queue)

        if current in visited:
            continue
        visited.add(current)

        if current == target_name:
            break

        current_city = graph.nodes[current]['city_data']
        neighbors = get_neighbors(cities, current_city)

        for neighbor in neighbors:
          if neighbor.name in visited:
              continue

          if neighbor.name not in graph:
              graph.add_node(neighbor.name, city_data=neighbor)

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

    # reconstructing the shortest path
    path = []
    current = target_name
    while current in previous:
        prev = previous[current]
        weight = graph[prev][current]['weight']
        path.append((current, weight))
        current = prev
    if current == start_name:
        path.append((start_name, 0))
        path.reverse()
        return path, distances[target_name]
    else:
        return None, float('inf')
    

def bellman_ford(graph: networkx.Graph, cities: list, start_name: str, target_name: str):
    distances = {start_name: 0}
    previous = {}
    visited = set()

    nodes = set([start_name])
    all_nodes = set([start_name])

    # max iterations = # of cities - 1 (bellman-ford constraint)
    for _ in range(len(cities) - 1):
        updated = False
        new_nodes = set()

        for u in list(nodes):
            if u in visited:
                continue
            visited.add(u)

            current_city = graph.nodes[u]['city_data']
            neighbors = get_neighbors(cities, current_city)

            for neighbor in neighbors:
                v = neighbor.name

                # add to graph if not present
                if v not in graph:
                    graph.add_node(v, city_data=neighbor)

                # add edge if missing
                if not graph.has_edge(u, v):
                    weight = haversine(current_city.latitude, current_city.longitude, neighbor.latitude, neighbor.longitude)
                    graph.add_edge(u, v, weight=weight)
                else:
                    weight = graph[u][v]['weight']

                if v not in distances:
                    distances[v] = float('inf')

                if distances[u] + weight < distances[v]:
                    distances[v] = distances[u] + weight
                    previous[v] = u
                    updated = True
                    new_nodes.add(v)
                    all_nodes.add(v)

        if not updated:
            break
        nodes = new_nodes

    # reconstruct path
    path = []
    current = target_name
    while current in previous:
        prev = previous[current]
        weight = graph[prev][current]['weight']
        path.append((current, weight))
        current = prev
    if current == start_name:
        path.append((start_name, 0))
        path.reverse()
        return path, distances[target_name]
    else:
        return None, float('inf')

def compare_algos(start_name: str, start_admin_name: str, target_name: str, file_name: str):
  
  cities = get_all_cities(start_name, start_admin_name, file_name)
  graph = build_graph(cities, 10000)
  print('\nusing dijkstra...')
  path, distance = dijkstra(graph, cities, start_name, target_name)
  print(f'shortest path: {path}')
  print(f'total distance: {distance:.2f} km\n')

  cities = get_all_cities(start_name, start_admin_name, file_name)
  graph = build_graph(cities, 10000)
  print('\nusing bellman-ford...')
  path, distance = bellman_ford(graph, cities, start_name, target_name)
  print(f'shortest path: {path}')
  print(f'total distance: {distance:.2f} km\n')

def main():
  print('\n----------- note: city names are case sensitive -----------')
  file_name = input('enter desired file name: ')
  start_name = input('enter starting city name: ')
  start_admin_name = input('enter starting city admin name (i.e. state name like florida): ')
  target_name = input('enter target city name: ')
  compare_algos(start_name, start_admin_name, target_name, file_name)

if __name__ == '__main__':
   main()