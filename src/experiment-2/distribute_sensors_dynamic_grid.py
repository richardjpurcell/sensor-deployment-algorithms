"""
distribute_sensors_dynamic_grid.py

Author: Richard Purcell  
Date: 09-01-24 
Description:  
    This script dynamically distributes sensors across a raster grid using a 
    level-based approach. The grid is divided into components based on raster 
    values, and the sensor placement adapts to the size and number of components 
    at each level. The goal is to evenly distribute sensors across areas of 
    interest while ensuring sufficient spatial coverage.

Key Features:
    - Reads a raster file to identify valid regions (levels) for sensor placement.
    - Divides each level into connected components using binary labeling.
    - Dynamically adjusts cell size based on component area and the target 
      number of sensors per component.
    - Outputs a shapefile containing sensor locations.

Dependencies:
    - rasterio
    - numpy
    - geopandas
    - shapely
    - scipy (for connected component analysis)

Usage:
    Run the script with specified file paths and parameters:
    - Input raster file (GeoTIFF format).
    - Desired total number of sensors.
    - Output shapefile for sensor points.

Example Command:
    python distribute_sensors_dynamic_grid.py

Parameters:
    - input_path: Path to the input raster file (e.g., "path/to/input/raster.tif").
    - output_path_points: Path to save the sensor points shapefile.
    - output_path_polygons: (Unused placeholder in current version).
    - n_points: Total number of sensors to distribute.

Outputs:
    - points_4_example.shp: Shapefile containing the distributed sensor points.

Notes:
    - Sensors are distributed by iterating through raster levels in descending order.
    - Connected components are identified within each level to ensure spatial balance.
    - The script adapts the cell size dynamically based on component area and target 
      sensor count, ensuring efficient sensor placement across varying regions.

"""

import math
import numpy as np
import rasterio
import geopandas as gpd
from shapely.geometry import Point
from scipy.ndimage import label, generate_binary_structure

def calculate_cell_size(area, num_components, points_per_component):
    if num_components == 0:
        return int(1e9)  # Very large number instead of infinity.
    area_per_component = area / num_components
    if points_per_component > 0:
        cell_size = math.sqrt(area_per_component / max(1, points_per_component))
    else:
        cell_size = float('inf')  # Avoid division by zero.
    return int(max(1, math.ceil(cell_size)))  # Ensure cell_size is at least 1.

def distribute_points_area_components(input_path, n_points, output_path_points, output_path_polygons):
    with rasterio.open(input_path) as src:
        raster_data = src.read(1)
        nodata = src.nodatavals[0]
        transform = src.transform
        crs = src.crs
        unique_levels = np.unique(raster_data[raster_data != nodata])
        points_per_level = max(1, n_points // len(unique_levels))  # Ensure at least one point per level.
        total_points = []
        total_points_count = 0
        max_points = int(n_points * 1.05)  # Allow up to 105% of n_points

        for level in sorted(unique_levels, reverse=True):
            if total_points_count >= max_points:
                break  # Stop if we've reached the maximum allowed points
            mask = (raster_data == level)
            structure = generate_binary_structure(2, 1)
            labeled, num_components = label(mask, structure)
            level_area = np.sum(mask)
            points_to_assign = min(points_per_level, max_points - total_points_count)
            points_per_component = max(1, points_to_assign // num_components)  # Ensure at least one point per component.

            points = []
            for component in range(1, num_components + 1):
                component_mask = labeled == component
                cell_size = calculate_cell_size(np.sum(component_mask), 1, points_per_component)
                for j in range(0, raster_data.shape[0], max(1, cell_size)):
                    for i in range(0, raster_data.shape[1], max(1, cell_size)):
                        if component_mask[j, i]:
                            x, y = transform * (i + 0.5, j + 0.5)
                            points.append(Point(x, y))

            points_assigned = np.random.choice(points, size=min(len(points), points_to_assign), replace=False)
            total_points_count += len(points_assigned)
            total_points.extend(points_assigned)

            print(f"Level: {level}, Area: {level_area}, Num Components: {num_components}, " \
                  f"Points Per Level: {points_per_level}, Points Per Component: {points_per_component}, " \
                  f"Cell Size: {cell_size}, Points Generated: {len(points_assigned)}")

        gdf_points = gpd.GeoDataFrame(geometry=total_points, crs=crs)
        gdf_points.to_file(output_path_points)
        print(f"Total Distributed Points: {total_points_count}, Target: {n_points}")

if __name__ == "__main__":
    input_path = '../../data/HRM_WUI_4_levels.tif'
    output_path_points = '../../results/HRM_GD_points.shp'
    output_path_polygons = '../../results/HRM_GD_polygons.shp'
    n_points = 200  # Total number of points to distribute
    distribute_points_area_components(input_path, n_points, output_path_points, output_path_polygons)
    print("Sensor distribution completed.")
