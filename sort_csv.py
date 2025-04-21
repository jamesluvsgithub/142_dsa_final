import csv

with open("worldcities.csv", 'r', newline='', encoding='utf-8') as infile:
    reader = csv.reader(infile)
    header = next(reader)
    rows = list(reader)

# sort by latitude (index 2) then longitude (index 3)
rows.sort(key=lambda x: (float(x[2]), float(x[3])))

with open("worldcities_sorted.csv", "w", newline='', encoding='utf-8') as outfile:
    writer = csv.writer(outfile)
    writer.writerow(header)
    writer.writerows(rows)