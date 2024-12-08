# **Wildfire Sensor Deployment Algorithms**
## Optimizing Sensor Placement for Early Wildfire Detection

### **Description**
This repository contains Python implementations of various algorithms used for **IoT sensor deployment** within the Wildland-Urban Interface (WUI). These algorithms aim to optimize sensor placement to maximize fire detection coverage using burn probability maps. The scripts accompany the Research Aptitude Defense (RAD) project for exploring adaptive approaches to wildfire monitoring.

---

### **Repository Contents**

| **Algorithm**               | **Script**                                 | **Description**                                                                                       |
|-----------------------------|-------------------------------------------|-------------------------------------------------------------------------------------------------------|
| **Grid-Based (GR)**         | `distribute_sensors_grid.py`             | Deploys sensors evenly at grid intersections across the WUI, adjusting grid cell size to match \( n \) sensors. |
| **Random (RA)**             | `distribute_sensors_random.py`           | Places \( n \) sensors randomly within valid areas of the WUI.                                         |
| **Random Weighted (RW)**    | `distribute_sensors_random_weighted.py`  | Randomly deploys sensors but weights placement probability based on burn probability (BP) values.      |
| **Greedy (GY)**             | `distribute_sensors_greedy.py`           | Iteratively places sensors in the highest BP regions, prioritizing areas with the greatest risk.       |
| **Genetic (GA)**            | `distribute_sensors_genetic.py`          | Optimizes sensor locations using a genetic algorithm to maximize fire detection coverage.             |
| **Genetic Weighted (GW)**   | `distribute_sensors_genetic_weighted.py` | Extends the genetic algorithm by prioritizing placement in high BP regions using a weighted fitness function. |
| **Dynamic Grid (DG)**       | `distribute_sensors_dynamic_grid.py`     | Adapts grid density to burn probability levels, ensuring higher sensor concentration in high-risk areas. |

---

### **Dependencies**
Ensure you have the following Python libraries installed:
- **rasterio**: For handling raster data (GeoTIFF files).
- **numpy**: For numerical computations.
- **geopandas**: For creating and saving geospatial data.
- **shapely**: For geometry manipulations.
- **scipy**: Used in connected component analysis.

Install dependencies with:
```bash
pip install rasterio numpy geopandas shapely scipy
```
### **Usage Instructions**
Each script distributes sensors using its respective algorithm. To run a script:

#### **Set Input Parameters**:
- Input raster file (e.g., `path/to/burn_probability_map.tif`).
- Desired number of sensors \( n \).
- Output path for sensor shapefile.

#### **Run the Script**:
Example for the Greedy Algorithm:
```bash
python distribute_sensors_greedy.py
```

### **Output**
Sensor locations are saved as shapefiles (e.g., `output_points.shp`).

---

### **Example Workflow**
Below is an example workflow using the **Dynamic Grid** algorithm:

```python
input_path = 'data/HRM_WUI_levels.tif'   # Input raster file
output_path_points = 'results/dynamic_grid_points.shp'
n_points = 200  # Total sensors to distribute

# Run the script
python distribute_sensors_dynamic_grid.py
```
### **Notes on Algorithms**
- **Grid-Based**: Ensures systematic and even coverage across the WUI.
- **Greedy**: Prioritizes placement in high-risk areas but does not account for spatial redundancy.
- **Genetic Weighted**: Iteratively improves sensor placement while prioritizing areas with higher BP.
- **Dynamic Grid**: Adaptive grid density ensures sensors are concentrated in high BP regions.

---

### **License**
This project is licensed under the MIT License. See `LICENSE` for details.

---

### **Contact**
For questions, issues, or collaborations, please contact:  
- **Author**: Richard Purcell  
- **Email**: richard.purcell@dal.ca  
- **Affiliation**: Dalhousie University, Computer Science Department.  
