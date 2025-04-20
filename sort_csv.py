import csv

# Load the data
with open("worldcities.csv", 'r', newline='', encoding='utf-8') as infile:
    reader = csv.reader(infile)
    header = next(reader)  # Save the header
    rows = list(reader)

# Sort by Latitude (index 2) then Longitude (index 3)
rows.sort(key=lambda x: (float(x[2]), float(x[3])))

# Write the sorted data to a new file
with open("worldcities_sorted.csv", "w", newline='', encoding='utf-8') as outfile:
    writer = csv.writer(outfile)
    writer.writerow(header)
    writer.writerows(rows)