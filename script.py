import csv
import math
import heapq
import networkx
from scipy.spatial import KDTree
import numpy as np
# csv from here (free) https://simplemaps.com/data/world-cities

class City:
  def __init__(self, name: str, admin_name: str, latitude: float, longitude: float, dist_from_start: int):
    self.name = name
    self.admin_name = admin_name 
    self.latitude = float(latitude)
    self.longitude = float(longitude)
    self.dist_from_start = dist_from_start

def get_all_cities(start_name: str, start_admin_name: str, file_name: str) -> list:
  cities = list()
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

def coords_to_xyz(lat, lon):
  lat_rad = math.radians(lat)
  lon_rad = math.radians(lon)
  x = math.cos(lat_rad) * math.cos(lon_rad)
  y = math.cos(lat_rad) * math.sin(lon_rad)
  z = math.sin(lat_rad)
  return (x, y, z)

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

neighbor_cache = dict()  # key: (name, admin_name), value: list of neighbors
'''
a neighbor cache is used to speed up lookup of edge weights; reduces computation
'''
def get_neighbors(cities, base_city):
  '''
  fuck
  '''
  EARTH_RADIUS_KM = 6371
  key = (base_city.name, base_city.admin_name)
  if key in neighbor_cache:
    return neighbor_cache[key]

  base_xyz = coords_to_xyz(base_city.latitude, base_city.longitude)

  # Calculate angular radius and convert to chord length
  angular_radius = 1000 / EARTH_RADIUS_KM  # 1000 km in radians
  chord_radius = 2 * math.sin(angular_radius / 2)

  # Query KDTree
  indices = tree.query_ball_point(base_xyz, chord_radius)
  neighbors = [cities[i] for i in indices if cities[i] != base_city]

  neighbor_cache[key] = neighbors
  return neighbors



def build_graph(cities: list) -> networkx.Graph:
  '''
  builds the initial graph (undirected, weighted) using the networkx library
  
  only adds vertices whose edges are < 1000 (city is within 1000 km from the start node)
  '''
  max_distance_km = 1000
  graph = networkx.Graph()
  cities_new = list()

  for city in cities:
    if city.dist_from_start <= max_distance_km:
      city_key = (city.name, city.admin_name)
      graph.add_node(city_key, city_data=city)
      cities_new.append(city)

  for i, city1 in enumerate(cities_new):
    for j in range(i + 1, len(cities_new)):  # cleaner loop
      city2 = cities_new[j]
      distance = haversine(city1.latitude, city1.longitude, city2.latitude, city2.longitude)
      if distance <= max_distance_km:
        key1 = (city1.name, city1.admin_name)
        key2 = (city2.name, city2.admin_name)
        graph.add_edge(key1, key2, weight=distance)

  return graph



def dijkstra(graph: networkx.Graph, cities: list, start_name: str, start_admin_name: str, target_name: str, target_admin_name: str):
    start = (start_name, start_admin_name)
    target = (target_name, target_admin_name)
    queue = [(0, start)]

    distances = {node: float('inf') for node in graph.nodes}
    distances[start] = 0

    previous = dict()
    visited = set()

    while queue:
        current_dist, current = heapq.heappop(queue)
        if current in visited:
            continue
        visited.add(current)

        if current == target:
            break

        current_city = graph.nodes[current]['city_data']
        neighbors = get_neighbors(cities, current_city)

        for neighbor in neighbors:
            neighbor_key = (neighbor.name, neighbor.admin_name)
            if neighbor_key in visited:
                continue

            if neighbor_key not in graph:
                graph.add_node(neighbor_key, city_data=neighbor)

            if not graph.has_edge(current, neighbor_key):
                graph.add_edge(current, neighbor_key, weight=haversine(
                    current_city.latitude, current_city.longitude,
                    neighbor.latitude, neighbor.longitude))

            distance = graph[current][neighbor_key]['weight']
            new_dist = current_dist + distance

            if new_dist < distances.get(neighbor_key, float('inf')):
                distances[neighbor_key] = new_dist
                previous[neighbor_key] = current
                heapq.heappush(queue, (new_dist, neighbor_key))

    # Reconstruct path
    path = []
    current = target
    while current in previous:
        prev = previous[current]
        weight = graph[prev][current]['weight']
        path.append((current, weight))
        current = prev
    if current == start:
        path.append((start, 0))
        path.reverse()
        return path, distances[target]
    else:
        return None, float('inf')

    

def bellman_ford(graph: networkx.Graph, cities: list, start_name: str, start_admin_name: str, target_name: str, target_admin_name: str):
  start_key = (start_name, start_admin_name)
  target_key = (target_name, target_admin_name)

  distances = {start_key: 0}
  previous = dict()
  visited = set()

  nodes = set([start_key])
  all_nodes = set([start_key])

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
        v = (neighbor.name, neighbor.admin_name)

        if v not in graph:
          graph.add_node(v, city_data=neighbor)

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
  current = target_key
  while current in previous:
    prev = previous[current]
    weight = graph[prev][current]['weight']
    path.append((current, weight))
    current = prev
  if current == start_key:
    path.append((start_key, 0))
    path.reverse()
    return path, distances[target_key]
  else:
    return None, float('inf')


def compare_algos(start_name: str, start_admin_name: str, target_name: str, target_admin_name: str, file_name: str):
  global tree, xyz_coords
  cities = get_all_cities(start_name, start_admin_name, file_name)
  xyz_coords = [coords_to_xyz(c.latitude, c.longitude) for c in cities]
  tree = KDTree(xyz_coords)
  graph = build_graph(cities)
  print('\nusing dijkstra...')
  path, distance = dijkstra(graph, cities, start_name, start_admin_name, target_name, target_admin_name)
  print(f'shortest path: {path}')
  print(f'total distance: {distance:.2f} km\n')

  neighbor_cache.clear()

  cities = get_all_cities(start_name, start_admin_name, file_name)
  xyz_coords = [coords_to_xyz(c.latitude, c.longitude) for c in cities]
  tree = KDTree(xyz_coords)
  graph = build_graph(cities)

  print('\nusing bellman-ford...')
  path, distance = bellman_ford(graph, cities, start_name, start_admin_name, target_name, target_admin_name)
  print(f'shortest path: {path}')
  print(f'total distance: {distance:.2f} km\n')

def main():
  print('\n----------- note: city names are case sensitive -----------')
  file_name = input('enter desired file name: ')
  start_name = input('enter starting city name: ')
  start_admin_name = input('enter starting city admin name (i.e. state name like florida): ')
  target_name = input('enter target city name: ')
  target_admin_name = input('enter target admin name: ')
  compare_algos(start_name, start_admin_name, target_name, target_admin_name, file_name)

if __name__ == '__main__':
   main()
