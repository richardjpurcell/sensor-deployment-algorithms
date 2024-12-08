"""
distribute_sensors_grid.py

Author: Richard Purcell
Date: 09-01-24
Description:
    This script distributes a specified number of sensors within a target area, 
    based on a raster grid. It uses a grid-based approach to generate candidate 
    points, validates their positions against the raster values (e.g., ignoring 
    NoData regions), and adjusts grid density dynamically to meet the target 
    number of sensors.

    The script outputs two files:
    1. A shapefile containing the validated sensor points.
    2. A shapefile containing sensor influence areas represented as buffers 
       around each point.

Key Features:
    - Dynamically adjusts the grid cell size to meet the desired number of points.
    - Excludes areas with invalid raster values (e.g., NoData or specified exclusions).
    - Outputs both sensor locations and buffered polygons to represent sensor 
      coverage/influence areas.

Dependencies:
    - rasterio
    - numpy
    - geopandas
    - shapely

Usage:
    Run the script with specified file paths and parameters:
    - Input raster file (GeoTIFF format)
    - Number of desired sensors
    - Minimum distance between sensors
    - Sensor radius for coverage

Example Command:
    python distribute_sensors_grid.py

Default Parameters:
    - Input raster: 'path/to/your/WUI_raster.tif'
    - Output points: 'path/to/output/validated_grid_sensors_points.shp'
    - Output polygons: 'path/to/output/validated_grid_sensors_polygons.shp'
    - Number of sensors: 200
    - Sensor radius: 200 meters

Output:
    - validated_grid_sensors_points.shp: Sensor points
    - validated_grid_sensors_polygons.shp: Buffered sensor areas

"""

import rasterio
import numpy as np
import geopandas as gpd
from shapely.geometry import Point
import itertools

def distribute_sensors_with_grid(input_path, n_points, output_path_points, output_path_polygons, min_distance, sensor_radius):
    with rasterio.open(input_path) as src:
        raster_data = src.read(1)
        nodata = src.nodatavals[0]
        transform = src.transform
        crs = src.crs

        target_points = n_points
        target_range = (0.9 * target_points, 1.1 * target_points)
        valid_points = []

        # Estimate initial cell size
        cell_area = (src.bounds.right - src.bounds.left) * (src.bounds.top - src.bounds.bottom) / target_points
        cell_size = np.sqrt(cell_area)

        found_valid_points = False
        while not found_valid_points:
            x_coords = np.arange(src.bounds.left, src.bounds.right, cell_size)
            y_coords = np.arange(src.bounds.bottom, src.bounds.top, cell_size)
            
            grid_points = [Point(x, y) for x in x_coords for y in y_coords]

            # Convert grid points to raster indices
            raster_indices = [(~transform * (point.x, point.y)) for point in grid_points]
            raster_indices = [(int(y), int(x)) for x, y in raster_indices if 0 <= int(x) < raster_data.shape[1] and 0 <= int(y) < raster_data.shape[0]]

            # Filter based on raster value
            valid_indices = [idx for idx in raster_indices if raster_data[idx] not in [nodata, -1]]
            valid_points = [Point(transform * (idx[1], idx[0])) for idx in valid_indices]

            if target_range[0] <= len(valid_points) <= target_range[1]:
                found_valid_points = True
            elif len(valid_points) < target_range[0]:
                cell_size *= 0.9  # Decrease cell size to increase points
            elif len(valid_points) > target_range[1]:
                cell_size *= 1.1  # Increase cell size to decrease points

        gdf_points = gpd.GeoDataFrame(geometry=valid_points, crs=crs)
        gdf_points.to_file(output_path_points)

        gdf_polygons = gpd.GeoDataFrame(geometry=[point.buffer(sensor_radius) for point in valid_points], crs=crs)
        gdf_polygons.to_file(output_path_polygons)

if __name__ == "__main__":
    input_path = '../../data/HRM_WUI.tif'
    output_path_points = '../../results/HRM_GR_points.shp'
    output_path_polygons = '../../results/HRM_GR__polygons.shp'
    n_points = 200
    min_distance = 2.5
    sensor_radius = 200

    distribute_sensors_with_grid(input_path, n_points, output_path_points, output_path_polygons, min_distance, sensor_radius)
    print("Sensor distribution with grid and validation against NA areas completed.")
