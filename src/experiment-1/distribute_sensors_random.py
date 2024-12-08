"""
distribute_sensors_random.py

Author: Richard Purcell  
Date: 09-01-24 
Description:  
    This script randomly distributes a specified number of sensors within valid 
    regions of a raster grid. The script identifies valid raster cells 
    (non-NoData regions), randomly selects points, and outputs both the 
    sensor locations and their coverage areas as shapefiles.

Key Features:
    - Reads a raster file to determine valid regions for sensor placement.
    - Randomly distributes sensors within valid regions.
    - Outputs two shapefiles:
        1. Sensor locations (points).
        2. Sensor coverage areas as circular polygons around each point.

Dependencies:
    - rasterio
    - numpy
    - geopandas
    - shapely

Usage:
    Run the script with appropriate file paths and parameters:
    - Input raster file (GeoTIFF format).
    - Desired number of sensors.
    - Sensor radius for coverage areas.

Example Command:
    python distribute_sensors_random.py

Parameters:
    - input_path: Path to the input raster file (e.g., "path/to/input/raster.tif").
    - output_path_points: Path to save the sensor points shapefile.
    - output_path_polygons: Path to save the sensor coverage polygons shapefile.
    - n_points: Number of sensors to distribute (default: 200).
    - min_distance: (Currently unused) Intended for minimum distance between sensors.
    - sensor_radius: Radius for the buffer zones (e.g., 200 meters).

Outputs:
    - points.shp: Shapefile containing sensor locations.
    - polygons.shp: Shapefile containing sensor coverage areas (buffered circles).

Notes:
    - The 'min_distance' parameter is defined but not yet implemented in this version.
    - The function assumes valid raster cells are those not equal to the NoData value.

"""

import rasterio
import numpy as np
import geopandas as gpd
from shapely.geometry import Point
import random

def distribute_sensors_random(input_path, n_points, output_path_points, output_path_polygons, min_distance, sensor_radius):
    with rasterio.open(input_path) as src:
        raster_data = src.read(1)
        nodata = src.nodatavals[0]
        transform = src.transform
        
        # Extract indices of all valid (non-nodata) cells
        valid_indices = np.argwhere(raster_data != nodata)
        
        # Randomly select n_points indices from the valid_indices
        selected_indices = random.sample(list(valid_indices), n_points)
        
        # Convert selected indices to geographic coordinates
        selected_coords = [transform * (idx[1], idx[0]) for idx in selected_indices]
        
        # Create GeoDataFrame with selected sensor locations
        gdf_points = gpd.GeoDataFrame(geometry=[Point(x, y) for x, y in selected_coords], crs=src.crs)
        
        # Save points to a shapefile
        gdf_points.to_file(output_path_points)
        
        # Create and save circular polygons centered on the points with a specified radius as a shapefile
        gdf_polygons = gpd.GeoDataFrame(geometry=[Point(x, y).buffer(sensor_radius) for x, y in selected_coords], crs=src.crs)
        gdf_polygons.to_file(output_path_polygons)

if __name__ == "__main__":
    # Example usage with parameters specified here
    input_path = '../../data/HRM_WUI.tif'
    output_path_points = '../../results/HRM_RA_points.shp'
    output_path_polygons = '../../results/HRM_RA_polygons.shp'
    n_points = 200  # Number of sensors to randomly distribute
    min_distance = 2.5  # This parameter is currently not used in the function
    sensor_radius = 200  # Radius for creating circular polygons around each point
    
    distribute_sensors_random(input_path, n_points, output_path_points, output_path_polygons, min_distance, sensor_radius)
    print("Random sensor distribution completed.")
