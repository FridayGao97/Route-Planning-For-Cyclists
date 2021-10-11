import osmnx as ox
from ipyleaflet import *
import networkx as nx
import plotly.graph_objects as obj
import numpy as np
import json
import requests

# Constants
nominatEIM_BASE_URL = 'https://nominatim.openstreetmap.org/search';


# Parameters
origin_addr = "832 Bay Street Toronto Ontario";
search_radius = 2000;
list_of_pois = ['park', 'school'];

print("Source point: ", origin_addr);
print("Search radius from source in meters: ", search_radius);
print("POI selection: ", list_of_pois);

# Get POI nodes
tags = {
	'amenity': list_of_pois
} 
POIs = ox.geometries.geometries_from_address(address=origin_addr, tags=tags, dist=search_radius)
print("POI nodes: ", POIs);

# Get the geocode of the origin address
params = {
    'q': origin_addr,
    'format': 'geocodejson'
}
res = requests.get(url=nominatEIM_BASE_URL, params=params);
origin_corrdinates = [];
if (res.status_code == 200):
    origin_corrdinates = res.json()['features'][0]['geometry']['coordinates'];
else:
    raise Exception("Failed to retrieve geocoding for origin");
print("Geocode of the source point: ", origin_corrdinates);

#Fetch Map from OSM, and find the closest node to origin
G = ox.graph.graph_from_address(address=origin_addr, dist=search_radius, dist_type='bbox', 
	network_type='bike', simplify=True, retain_all=False, truncate_by_edge=False, 
	return_coords=False, clean_periphery=True, custom_filter=None);
fig, ax = ox.plot_graph(G);
origin_node_id = ox.get_nearest_node(G, origin_corrdinates);
nodes, edges = ox.graph_to_gdfs(G);