"""
distribute_sensors_genetic.py

Author: Richard Purcell  
Date: 09-01-24
Description:  
    This script optimizes the placement of sensors within a geographic area using a 
    Genetic Algorithm (GA). By considering a raster map with varying levels of 
    importance, the algorithm finds an optimal sensor configuration that maximizes 
    coverage while adhering to spatial constraints such as minimum distance 
    between sensors.

Key Features:
    - Reads a raster file to identify valid regions for sensor placement.
    - Uses a Genetic Algorithm to iteratively evolve and optimize sensor positions:
        - Fitness is based on coverage of high-importance areas and penalties for 
          placing sensors too close together.
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
    - Sensor radius for coverage.

Example Command:
    python distribute_sensors_genetic.py

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
    - Fitness evaluation considers both the importance levels of raster cells and 
      penalties for sensors violating the minimum distance constraint.
    - The Genetic Algorithm evolves sensor positions over a defined number of 
      generations to achieve near-optimal coverage.

"""


import rasterio
import numpy as np
import geopandas as gpd
from shapely.geometry import Point, Polygon
import random
from shapely.geometry import Point
from pyproj import Proj, transform
from pyproj import CRS, Transformer


def evaluate_fitness(individual, raster_data, level_importance, min_distance, transformer, proj):
    # Calculate base fitness based on level importance
    levels = np.array([raster_data[point[0], point[1]] for point in individual])
    fitness = sum(level_importance[level] for level in levels)
    
    # Penalty for points too close to each other
    penalty = 0
    for i, point1 in enumerate(individual):
        for point2 in individual[i+1:]:
            if calculate_distance(point1, point2, transformer) < min_distance:
                penalty += 1  # Increment penalty for each pair of points too close
                
    # Subtract penalty from fitness
    fitness -= penalty
    return fitness

'''
def calculate_distance(point1, point2):
    # Euclidean distance between two points
    return np.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)
    
    # Create Shapely Points and calculate distance in meters
    shapely_point1 = Point(x1, y1)
    shapely_point2 = Point(x2, y2)
    distance = shapely_point1.distance(shapely_point2)
    
    return distance
'''
def calculate_distance(point1, point2, transformer):
    # Transform points
    x1, y1 = transformer.transform(point1[1], point1[0])
    x2, y2 = transformer.transform(point2[1], point2[0])
    
    # Calculate Euclidean distance
    distance = ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5
    return distance


def initialize_population(n_points, valid_cells, population_size=50):
    population = []
    for _ in range(population_size):
        individual = random.choices(valid_cells, k=n_points)
        population.append(individual)
    return population

def select_parents(population, fitness_scores):
    parents = sorted(zip(population, fitness_scores), key=lambda x: x[1], reverse=True)
    return [parent[0] for parent in parents[:2]]

def crossover_and_mutate(parent1, parent2, valid_cells, mutation_rate=0.05):
    cut = len(parent1) // 2
    offspring = parent1[:cut] + parent2[cut:]
    if random.random() < mutation_rate:
        mutation_idx = random.randrange(len(offspring))
        offspring[mutation_idx] = random.choice(valid_cells)
    return offspring

def genetic_algorithm(raster_path, n_points=100, population_size=50, num_generations=100, min_distance=2.5):
    with rasterio.open(raster_path) as src:
        raster_data = src.read(1)
        no_data = src.nodatavals[0]
        transform = src.transform
        valid_cells = [(row, col) for row in range(raster_data.shape[0])
                       for col in range(raster_data.shape[1])
                       if raster_data[row, col] != no_data]
        
        level_importance = {0: 1, 1: 2, 2: 3, 3: 4, 4: 5}
        population = initialize_population(n_points, valid_cells, population_size)

        transformer = Transformer.from_crs(src.crs, src.crs, always_xy=True)
        
        for _ in range(num_generations):
            fitness_scores = [evaluate_fitness(ind, raster_data, level_importance, min_distance, transformer, src.crs) for ind in population]
            parents = select_parents(population, fitness_scores)
            population = [crossover_and_mutate(parents[0], parents[1], valid_cells) for _ in range(population_size)]
        
        best_individual = max(population, key=lambda ind: evaluate_fitness(ind, raster_data, level_importance, min_distance, transformer, src.crs))
        best_points = [transform * (col, row) for row, col in best_individual]
        return best_points, src.crs

def save_points_as_shapefile(points, crs, output_path_points):
    """Saves the generated points as a shapefile."""
    gdf_points = gpd.GeoDataFrame(geometry=[Point(x, y) for x, y in points], crs=crs)
    gdf_points.to_file(output_path_points)

def save_polygons_as_shapefile(points, crs, output_path_polygons, radius=200):
    """Creates and saves circular polygons centered on the points with a specified radius as a shapefile."""
    gdf_polygons = gpd.GeoDataFrame(geometry=[Point(x, y).buffer(radius) for x, y in points], crs=crs)
    gdf_polygons.to_file(output_path_polygons)

# This function now accepts additional parameters to control the genetic algorithm and sensor deployment.
def run_optimization_and_save_results(input_path, output_path_points, output_path_polygons, n_points, population_size, num_generations, min_distance, sensor_radius):
    points, crs = genetic_algorithm(input_path, n_points, population_size, num_generations, min_distance)
    save_points_as_shapefile(points, crs, output_path_points)
    save_polygons_as_shapefile(points, crs, output_path_polygons, sensor_radius)

if __name__ == "__main__":
    # Example usage with parameters specified here
    input_path = '../../data/HRM_WUI.tif'
    output_path_points = '../../results/HRM_GA_points.shp'
    output_path_polygons = '../../results/HRM_GA_polygons.shp'
    n_points = 200
    population_size = 50
    num_generations = 100
    min_distance = 2.5
    sensor_radius = 200
    
    run_optimization_and_save_results(input_path, output_path_points, 
                                      output_path_polygons, n_points, 
                                      population_size, num_generations, 
                                      min_distance, sensor_radius)

