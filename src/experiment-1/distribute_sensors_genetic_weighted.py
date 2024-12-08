"""
distribute_sensors_genetic_weighted.py

Author: Richard Purcell  
Date: 09-01-24 
Description:  
    This script optimizes the placement of sensors within a geographic area using a 
    **weighted Genetic Algorithm (GA)**. Sensor positions are optimized based on raster 
    cell values, where higher values are given more importance in the selection process. 
    The goal is to maximize sensor coverage of high-importance areas while maintaining 
    spatial constraints such as minimum distance between sensors.

Key Features:
    - Reads a raster file to identify valid regions for sensor placement.
    - Assigns weights based on raster values to prioritize high-importance areas.
    - Uses a Genetic Algorithm to iteratively optimize sensor placement:
        - Fitness is inversely proportional to penalties for sensors violating minimum 
          distance constraints.
    - Outputs two shapefiles:
        1. Optimal sensor locations (points).
        2. Sensor coverage areas (buffered circles around points).

Dependencies:
    - rasterio
    - numpy
    - geopandas
    - shapely
    - pyproj

Usage:
    Run the script with specified file paths and parameters:
    - Input raster file (GeoTIFF format).
    - Number of sensors to place.
    - Genetic Algorithm parameters (population size, generations, mutation rate).
    - Sensor radius for coverage areas.

Example Command:
    python distribute_sensors_genetic_weighted.py

Parameters:
    - input_path: Path to the input raster file (e.g., "path/to/input/raster.tif").
    - output_path_points: Path to save the optimized sensor points shapefile.
    - output_path_polygons: Path to save the coverage polygons shapefile.
    - n_points: Number of sensors to optimize placement for (default: 200).
    - population_size: Size of the genetic algorithm population (default: 50).
    - num_generations: Number of generations for the genetic algorithm (default: 1000).
    - min_distance: Minimum allowable distance between sensors (e.g., 2.5 units).
    - sensor_radius: Radius for the buffer zones representing sensor coverage.

Outputs:
    - points.shp: Shapefile containing the optimized sensor locations.
    - polygons.shp: Shapefile containing the coverage areas as buffered circles.

Notes:
    - The fitness function prioritizes high-importance raster cells (weighted regions).
    - Penalties are applied for sensors violating the minimum distance constraint.
    - The Genetic Algorithm evolves sensor positions over defined generations to achieve 
      optimal coverage while adhering to constraints.

"""

import rasterio
import numpy as np
import geopandas as gpd
from shapely.geometry import Point
import random
from pyproj import CRS, Transformer

def evaluate_fitness(individual, raster_data, min_distance, transformer):
    # Penalty for points too close to each other
    penalty = 0
    for i, point1 in enumerate(individual):
        for j, point2 in enumerate(individual):
            if i != j and calculate_distance(point1, point2, transformer) < min_distance:
                penalty += 1  # Increment penalty for each pair of points too close
                
    # Fitness is inversely proportional to the penalty
    fitness = -penalty
    return fitness

def calculate_distance(point1, point2, transformer):
    # Transform points to geographic coordinates
    x1, y1 = transformer.transform(point1[1], point1[0])
    x2, y2 = transformer.transform(point2[1], point2[0])
    
    # Calculate Euclidean distance
    distance = np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
    return distance

def initialize_population(n_points, valid_cells, population_size):
    population = []
    for _ in range(population_size):
        individual = random.sample(valid_cells, k=n_points)
        population.append(individual)
    return population

def select_parents(population, fitness_scores):
    parents = sorted(zip(population, fitness_scores), key=lambda x: x[1], reverse=True)[:2]
    return [parent[0] for parent in parents]

def crossover_and_mutate(parent1, parent2, valid_cells, mutation_rate=0.05):
    cut_point = len(parent1) // 2
    offspring = parent1[:cut_point] + parent2[cut_point:]
    if random.random() < mutation_rate:
        mutation_index = random.randrange(len(offspring))
        offspring[mutation_index] = random.choice(valid_cells)
    return offspring

def genetic_algorithm(raster_path, n_points=100, population_size=50, num_generations=100, min_distance=2.5):
    with rasterio.open(raster_path) as src:
        raster_data = src.read(1)
        no_data = src.nodatavals[0]
        transform = src.transform
        transformer = Transformer.from_crs(src.crs, src.crs, always_xy=True)

        # Exclude NA or -1 values
        valid_cells = [(row, col) for row in range(raster_data.shape[0])
                       for col in range(raster_data.shape[1])
                       if raster_data[row, col] not in [no_data, -1]]

        population = initialize_population(n_points, valid_cells, population_size)

        for _ in range(num_generations):
            fitness_scores = [evaluate_fitness(individual, raster_data, min_distance, transformer) for individual in population]
            parents = select_parents(population, fitness_scores)
            population = [crossover_and_mutate(parents[0], parents[1], valid_cells, mutation_rate=0.05) for _ in range(population_size)]

        best_individual = max(population, key=lambda individual: evaluate_fitness(individual, raster_data, min_distance, transformer))
        best_points = [transform * (col, row) for row, col in best_individual]
        return best_points, src.crs

def save_points_as_shapefile(points, crs, output_path_points):
    gdf_points = gpd.GeoDataFrame(geometry=[Point(x, y) for x, y in points], crs=crs)
    gdf_points.to_file(output_path_points)

def save_polygons_as_shapefile(points, crs, output_path_polygons, radius=200):
    gdf_polygons = gpd.GeoDataFrame(geometry=[Point(x, y).buffer(radius) for x, y in points], crs=crs)
    gdf_polygons.to_file(output_path_polygons)

def run_optimization_and_save_results_GB(input_path, output_path_points, output_path_polygons, n_points, population_size, num_generations, min_distance, sensor_radius):
    points, crs = genetic_algorithm(input_path, n_points, population_size, num_generations, min_distance)
    save_points_as_shapefile(points, crs, output_path_points)
    save_polygons_as_shapefile(points, crs, output_path_polygons, sensor_radius)

if __name__ == "__main__":
    # Set your file paths and parameters for the example usage
    input_path = '../../data/HRM_WUI_255_levels.tif'
    output_path_points = '../../results/HRM_GW_points.shp'
    output_path_polygons = '../../results/HRM_GW_polygons.shp'
    n_points = 200
    population_size = 50
    num_generations = 100
    min_distance = 2.5
    sensor_radius = 200
    
    run_optimization_and_save_results_GB(input_path, output_path_points, output_path_polygons, n_points, population_size, num_generations, min_distance, sensor_radius)
