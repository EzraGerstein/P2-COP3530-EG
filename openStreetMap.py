import osmnx as ox

# 1. Configure settings safely
ox.settings.use_cache = True
ox.settings.log_console = True

# 2. Coordinates centered near the UF Reitz Union
uf_center = (29.6465, -82.3433)

# 3. Broaden tags to catch parks, gardens, and common campus greens near UF
tags = {
    "leisure": ["park", "garden", "common"],
    "landuse": ["recreation_ground", "grass"]
}

# 4. Retrieve features within 2500 meters
try:
    parks_gdf = ox.features.features_from_point(uf_center, tags=tags, dist=2500)
except Exception as e:
    print(f"Error fetching data from OSM: {e}")
    parks_gdf = None

# 5. Process and display results safely
if parks_gdf is not None and not parks_gdf.empty:
    # Ensure a 'name' column exists, fill missing names with 'Unnamed Area' instead of dropping rows
    if "name" not in parks_gdf.columns:
        parks_gdf["name"] = "Unnamed Area"
    else:
        parks_gdf["name"] = parks_gdf["name"].fillna("Unnamed Area")

    print(f"Found {len(parks_gdf)} matching features!")
    print(parks_gdf[["name", "geometry"]].head(10))
else:
    print("No features found matching the given tags within the specified distance.")