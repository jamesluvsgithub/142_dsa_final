import csv
# csv from here (free) https://simplemaps.com/data/world-cities

class City:
  def __init__(self, name, latitude, longitude):
    self.name = name
    self.latitude = float(latitude)
    self.longitude = float(longitude)

cities = []

with open("worldcities.csv", newline="", encoding="utf-8") as csvfile:
  reader = csv.DictReader(csvfile)
  for row in reader:
    name = row["city"]
    lat = float(row["lat"])
    lng = float(row["lng"])
    cities.append(City(name, lat, lng))

for city in cities[:10]:
  print(city.name, city.latitude, city.longitude)