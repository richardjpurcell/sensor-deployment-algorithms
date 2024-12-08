"""
distribute_sensors_random_weighted.py

Author: Richard Purcell  
Date: 09-01-24  
Description:  
    This script randomly distributes a specified number of sensors within valid 
    regions of a raster grid, using weighted probabilities. The weights are based 
    on raster cell values, allowing areas with higher values to have a higher 
    chance of being selected. The script outputs both the sensor locations and 
    their coverage areas as shapefiles.

Key Features:
    - Reads a raster file to determine valid regions for sensor placement.
    - Assigns selection probabilities (weights) based on raster cell values.
    - Randomly distributes sensors using weighted probabilities.
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
    python distribute_sensors_random_weighted.py

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
    - Selection weights are assigned based on raster cell values using a predefined map.
    - Undefined raster values default to a weight of 1.
    - The 'min_distance' parameter is defined but not yet implemented in this version.

"""

import rasterio
import numpy as np
import geopandas as gpd
from shapely.geometry import Point

def distribute_sensors_weighted(input_path, n_points, output_path_points, output_path_polygons, min_distance, sensor_radius):
    with rasterio.open(input_path) as src:
        raster_data = src.read(1)
        nodata = src.nodatavals[0]
        transform = src.transform
        
        # Ensure we only consider non-nodata cells
        valid_cells = np.argwhere(raster_data != nodata)
        
        # Create a weights array based on raster values, with a default weight for undefined values
        default_weight = 1
        weights_map = {0: 1, 1: 2, 2: 4, 3: 8, 4: 16}
        weights = np.array([
            weights_map.get(raster_data[row, col], default_weight) 
            for row, col in valid_cells
        ])
        
        # Normalize weights
        weights = weights / np.sum(weights)
        
        # Randomly select indices from valid_cells based on the weights
        chosen_indices = np.random.choice(len(valid_cells), size=n_points, replace=False, p=weights)
        
        # Convert chosen indices to geographic coordinates
        selected_coords = [transform * (valid_cells[idx][1], valid_cells[idx][0]) for idx in chosen_indices]
        
        # Create GeoDataFrame for points
        gdf_points = gpd.GeoDataFrame(geometry=[Point(x, y) for x, y in selected_coords], crs=src.crs)
        gdf_points.to_file(output_path_points)
        
        # Create and save circular polygons for polygons
        gdf_polygons = gpd.GeoDataFrame(geometry=[Point(x, y).buffer(sensor_radius) for x, y in selected_coords], crs=src.crs)
        gdf_polygons.to_file(output_path_polygons)

if __name__ == "__main__":
    # Set your file paths and parameters
    input_path = '../../data/HRM_WUI_255_levels.tif'  # Replace with your raster file path
    output_path_points = '../../results/HRM_RW_points.shp'
    output_path_polygons = '../../results/HRM_RW_polygons.shp'
    n_points = 200
    min_distance = 2.5  # Currently unused
    sensor_radius = 200

    distribute_sensors_weighted(input_path, n_points, output_path_points, output_path_polygons, min_distance, sensor_radius)
    print("Weighted sensor distribution completed.")
