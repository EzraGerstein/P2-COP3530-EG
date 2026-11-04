import sys
import os
import json
import requests
import numpy as np
from PIL import Image, ImageDraw

def fetch_or_load_osm_data():
    cache_candidates = [
        "assets/uf_osm_cache.json",
        "../assets/uf_osm_cache.json",
        "uf_osm_cache.json"
    ]
    for c_path in cache_candidates:
        if os.path.exists(c_path):
            print(f"Loading cached map data from {c_path}")
            with open(c_path, "r", encoding="utf-8") as f:
                return json.load(f)

    print("Fetching live map data for UF campus from Overpass API...")
    # Bounding box around Lake Alice and trails
    min_lat, max_lat = 29.6380, 29.6520
    min_lon, max_lon = -82.3660, -82.3460

    query = f"""
    [out:json][timeout:45];
    (
      way["natural"="water"]({min_lat},{min_lon},{max_lat},{max_lon});
      way["water"]({min_lat},{min_lon},{max_lat},{max_lon});
      relation["natural"="water"]({min_lat},{min_lon},{max_lat},{max_lon});
      relation["water"]({min_lat},{min_lon},{max_lat},{max_lon});
      way["landuse"="grass"]({min_lat},{min_lon},{max_lat},{max_lon});
      way["leisure"~"park|garden"]({min_lat},{min_lon},{max_lat},{max_lon});
      way["highway"~"residential|tertiary|secondary|service|primary|trunk"]({min_lat},{min_lon},{max_lat},{max_lon});
      way["building"]({min_lat},{min_lon},{max_lat},{max_lon});
      way["highway"~"footway|path|pedestrian|cycleway"]({min_lat},{min_lon},{max_lat},{max_lon});
    );
    out geom;
    """

    headers = {"User-Agent": "TrailQuest-UF-COP3530/1.0 (academic research)"}
    endpoints = [
        "https://overpass-api.de/api/interpreter",
        "https://overpass.kumi.systems/api/interpreter"
    ]

    for ep in endpoints:
        try:
            print(f"Querying {ep}...")
            r = requests.post(ep, data={"data": query}, headers=headers, timeout=50)
            if r.status_code == 200:
                data = r.json()
                os.makedirs("assets", exist_ok=True)
                with open("assets/uf_osm_cache.json", "w", encoding="utf-8") as f:
                    json.dump(data, f)
                print("Downloaded and saved to assets/uf_osm_cache.json")
                return data
        except Exception as e:
            print(f"Endpoint {ep} error: {e}")

    raise RuntimeError("Failed to download OSM data and no local cache was found.")

def generate_real_campus_grid(grid_size=100):
    data = fetch_or_load_osm_data()
    elements = data.get("elements", [])
    print(f"Loaded {len(elements)} OSM features")

    min_lat, max_lat = 29.6380, 29.6520
    min_lon, max_lon = -82.3660, -82.3460

    GRID_W, GRID_H = grid_size, grid_size

    def to_xy(lat, lon):
        x = int((lon - min_lon) / (max_lon - min_lon) * (GRID_W - 1))
        y = int((max_lat - lat) / (max_lat - min_lat) * (GRID_H - 1))
        return (max(0, min(GRID_W - 1, x)), max(0, min(GRID_H - 1, y)))

    # default everything to grass (0)
    img = Image.new("L", (GRID_W, GRID_H), 0)
    draw = ImageDraw.Draw(img)

    # 1. water (Lake Alice and ponds)
    for el in elements:
        tags = el.get("tags", {})
        is_water = ("natural" in tags and tags["natural"] == "water") or ("water" in tags)
        if not is_water:
            continue
        if el["type"] == "way":
            geom = el.get("geometry", [])
            if len(geom) >= 3:
                draw.polygon([to_xy(p["lat"], p["lon"]) for p in geom], fill=4)
        elif el["type"] == "relation":
            for m in el.get("members", []):
                if m.get("role") in ["outer", ""]:
                    geom = m.get("geometry", [])
                    if len(geom) >= 3:
                        draw.polygon([to_xy(p["lat"], p["lon"]) for p in geom], fill=4)
            # lake islands back to grass
            for m in el.get("members", []):
                if m.get("role") == "inner":
                    geom = m.get("geometry", [])
                    if len(geom) >= 3:
                        draw.polygon([to_xy(p["lat"], p["lon"]) for p in geom], fill=0)

    # 2. paved roads
    for el in elements:
        if "highway" in el.get("tags", {}) and el["tags"]["highway"] not in ["footway", "path", "pedestrian", "cycleway"]:
            geom = el.get("geometry", [])
            if len(geom) >= 2:
                draw.line([to_xy(p["lat"], p["lon"]) for p in geom], fill=2, width=1)

    # 3. buildings
    for el in elements:
        if "building" in el.get("tags", {}):
            geom = el.get("geometry", [])
            if len(geom) >= 3:
                draw.polygon([to_xy(p["lat"], p["lon"]) for p in geom], fill=3)

    # 4. trails and footpaths
    for el in elements:
        if el.get("tags", {}).get("highway") in ["footway", "path", "pedestrian", "cycleway"]:
            geom = el.get("geometry", [])
            if len(geom) >= 2:
                draw.line([to_xy(p["lat"], p["lon"]) for p in geom], fill=1, width=1)

    arr = np.array(img)

    # terrain weights matching C++
    weights = {
        0: 2.5,   # Grass
        1: 1.0,   # Trails
        2: 8.0,   # Roads
        3: 60.0,  # Buildings
        4: 99.0   # Water
    }

    names = {
        0: "Campus Grass",
        1: "Nature Trails",
        2: "Paved Roads",
        3: "Buildings",
        4: "Lake Alice (Water)"
    }

    total_cells = GRID_W * GRID_H
    print(f"Generated {GRID_W}x{GRID_H} campus grid ({total_cells} nodes)")
    for code in sorted(weights.keys()):
        cnt = int(np.sum(arr == code))
        pct = cnt * 100.0 / total_cells
        print(f"  {names[code]}: {cnt} cells ({pct:.1f}%) [weight {weights[code]}]")

    output_files = ["grid.txt", "cmake-build-debug/grid.txt"]
    for out_path in output_files:
        dir_name = os.path.dirname(out_path)
        if dir_name:
            os.makedirs(dir_name, exist_ok=True)
        with open(out_path, "w", encoding="utf-8") as f:
            f.write(f"{GRID_W} {GRID_H}\n")
            for y in range(GRID_H):
                for x in range(GRID_W):
                    idx = y * GRID_W + x
                    code = int(arr[y, x])
                    w = weights[code]
                    f.write(f"{idx} {x} {y} {w}\n")
        print(f"Saved grid to {out_path}")

if __name__ == "__main__":
    size = 100
    if len(sys.argv) > 1:
        try:
            size = int(sys.argv[1])
        except ValueError:
            pass
    generate_real_campus_grid(size)
