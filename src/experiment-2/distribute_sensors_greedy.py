"""
distribute_sensors_greedy.py

Author: Richard Purcell  
Date: [Insert Date]  
Description:  
    This script distributes sensors across a raster grid using a straightforward 
    greedy algorithm. Sensors are placed iteratively in the highest-value regions 
    (levels) of a raster grid until the desired number of sensors is reached.

Key Features:
    - Reads a raster file to identify valid regions (levels) for sensor placement.
    - Sensors are placed starting from the highest-value regions, avoiding duplicate 
      placements, until the total number of sensors is reached.
    - Outputs a shapefile containing sensor locations.

Dependencies:
    - rasterio
    - numpy
    - geopandas
    - shapely

Usage:
    Run the script with specified file paths and parameters:
    - Input raster file (GeoTIFF format).
    - Desired total number of sensors.
    - Output shapefile for sensor points.

Example Command:
    python distribute_sensors_greedy.py

Parameters:
    - input_path: Path to the input raster file (e.g., "path/to/input/raster.tif").
    - output_path_points: Path to save the sensor points shapefile.
    - n_points: Total number of sensors to distribute.

Outputs:
    - greedy_points.shp: Shapefile containing the distributed sensor points.

Notes:
    - Sensors are greedily placed in descending order of raster cell values.
    - The script ensures a fixed number of points is distributed without spatial 
      constraints like minimum distance.

"""

import rasterio
import numpy as np
import geopandas as gpd
from shapely.geometry import Point

def distribute_sensors_greedy(input_path, n_points, output_path_points):
    with rasterio.open(input_path) as src:
        raster_data = src.read(1)
        nodata = src.nodatavals[0]
        transform = src.transform
        crs = src.crs

        # Flatten raster and filter valid cells (exclude NoData values)
        valid_indices = np.argwhere(raster_data != nodata)
        valid_values = raster_data[valid_indices[:, 0], valid_indices[:, 1]]

        # Sort valid cells by descending raster values (greedy selection)
        sorted_indices = valid_indices[np.argsort(-valid_values)]

        # Select the top n_points cells
        selected_indices = sorted_indices[:n_points]
        total_points = []

        for row, col in selected_indices:
            x, y = transform * (col + 0.5, row + 0.5)  # Center of the cell
            total_points.append(Point(x, y))

        print(f"Greedy algorithm placed {len(total_points)} sensors.")

        # Save points as a shapefile
        gdf_points = gpd.GeoDataFrame(geometry=total_points, crs=crs)
        gdf_points.to_file(output_path_points)
        print(f"Sensor points saved to: {output_path_points}")

if __name__ == "__main__":
    input_path = '../../data/HRM_WUI.tif'
    output_path_points = '../../results/HRM_GY_points.shp'
    n_points = 800  # Total number of points to distribute

    distribute_sensors_greedy(input_path, n_points, output_path_points)
    print("Greedy sensor distribution completed.")
