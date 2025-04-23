import tkinter as tk
from tkinter import ttk, messagebox
from PIL import Image, ImageTk
import os
import folium
import webbrowser
from script import get_all_cities, build_graph, dijkstra, bellman_ford, coords_to_xyz, KDTree, neighbor_cache

def visualize_path(path):
    if not path:
        return

    coords = [(city.latitude, city.longitude) for city, _ in path]
    m = folium.Map(location=coords[0], zoom_start=5)

    folium.PolyLine(coords, color="red", weight=5, opacity=0.8).add_to(m)
    folium.Marker(coords[0], popup="Start", icon=folium.Icon(color="green")).add_to(m)
    folium.Marker(coords[-1], popup="End", icon=folium.Icon(color="red")).add_to(m)

    for lat, lon in coords[1:-1]:
        folium.CircleMarker(location=(lat, lon), radius=4, color='blue').add_to(m)

    filepath = "path_visualization.html"
    m.save(filepath)
    webbrowser.open(filepath)

def run_algorithm():
    algo = algo_var.get()
    start = start_entry.get().strip()
    start_admin = start_admin_entry.get().strip()
    end = end_entry.get().strip()
    end_admin = end_admin_entry.get().strip()

    if not (algo and start and end and start_admin and end_admin):
        messagebox.showerror("Missing Info", "Please fill in all fields.")
        return

    try:
        cities = get_all_cities(start, start_admin, "worldcities.csv")
        xyz_coords = [coords_to_xyz(c.latitude, c.longitude) for c in cities]
        tree = KDTree(xyz_coords)
        neighbor_cache = {}

        graph = build_graph(cities)
    except Exception as e:
        messagebox.showerror("Error", f"Failed to build graph:\n{e}")
        return

    try:
        if algo == "Dijkstra":
            path, distance = dijkstra(graph, cities, start, start_admin, end, end_admin, tree, neighbor_cache)
        else:
            path, distance = bellman_ford(graph, cities, start, end, tree, neighbor_cache)
    except Exception as e:
        messagebox.showerror("Algorithm Error", f"Something went wrong:\n{e}")
        return

    if not path:
        result_var.set("No path found.")
        return

    city_path = []
    for name, weight in path:
        try:
            city_obj = graph.nodes[name]["city_data"]
            city_path.append((city_obj, weight))
        except KeyError:
            continue

    path_str = " → ".join(f"{city.name} ({int(weight)})" for city, weight in city_path)
    result_var.set(f"Path: {path_str}\nTotal Distance: {distance:.2f} km")

    visualize_path(city_path)

# ------------------ gui setup ------------------
root = tk.Tk()
root.title("City Path Finder")
root.state("zoomed")

root.columnconfigure(0, weight=1)
root.columnconfigure(1, weight=1)
root.rowconfigure(0, weight=1)

# LEFT: Background Image
left_canvas = tk.Canvas(root, highlightthickness=0)
left_canvas.grid(row=0, column=0, sticky="nsew")

script_dir = os.path.dirname(os.path.abspath(__file__))
bg_image_path = os.path.join(script_dir, "background.jpg")
bg_image = Image.open(bg_image_path)

def resize_bg(event):
    new_width = event.width
    new_height = event.height
    resized = bg_image.resize((new_width, new_height), Image.Resampling.LANCZOS)
    bg_photo = ImageTk.PhotoImage(resized)
    left_canvas.bg_img = bg_photo
    left_canvas.delete("all")
    left_canvas.create_image(0, 0, image=bg_photo, anchor="nw")

left_canvas.bind("<Configure>", resize_bg)

# -------------- blue panel ---------------------------------------------
right_frame = tk.Frame(root, bg="#d0e6ff")
right_frame.grid(row=0, column=1, sticky="nsew")

content = tk.Frame(right_frame, bg="#d0e6ff")
content.pack(fill="both", expand=True, padx=40, pady=40)

for i in range(12):
    content.rowconfigure(i, weight=1)
content.columnconfigure(0, weight=1)
# -----------------------------------------------------------------------

algo_var = tk.StringVar(value="Dijkstra")
result_var = tk.StringVar()

# drop-down, text boxes, and button
ttk.Label(content, text="Algorithm:", background="#d0e6ff").grid(row=0, column=0, sticky="w", pady=5)
algo_menu = ttk.Combobox(content, textvariable=algo_var, values=["Dijkstra", "Bellman-Ford"], state="readonly", width=40)
algo_menu.grid(row=1, column=0, sticky="w", pady=5)

ttk.Label(content, text="Start City:", background="#d0e6ff").grid(row=2, column=0, sticky="w", pady=5)
start_entry = ttk.Entry(content, width=42)
start_entry.grid(row=3, column=0, sticky="w", pady=5)

ttk.Label(content, text="Start State/Region (admin_name):", background="#d0e6ff").grid(row=4, column=0, sticky="w", pady=5)
start_admin_entry = ttk.Entry(content, width=42)
start_admin_entry.grid(row=5, column=0, sticky="w", pady=5)

ttk.Label(content, text="Destination City:", background="#d0e6ff").grid(row=6, column=0, sticky="w", pady=5)
end_entry = ttk.Entry(content, width=42)
end_entry.grid(row=7, column=0, sticky="w", pady=5)

ttk.Label(content, text="Destination State/Region (admin_name):", background="#d0e6ff").grid(row=8, column=0, sticky="w", pady=5)
end_admin_entry = ttk.Entry(content, width=42)
end_admin_entry.grid(row=9, column=0, sticky="w", pady=5)

ttk.Button(content, text="Run Algorithm", command=run_algorithm).grid(row=10, column=0, pady=15, sticky="w")
ttk.Label(content, textvariable=result_var, wraplength=600, justify="left", foreground="blue", background="#d0e6ff").grid(row=11, column=0, sticky="w", pady=5)

root.mainloop()
