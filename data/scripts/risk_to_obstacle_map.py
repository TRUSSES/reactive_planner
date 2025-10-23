import pandas as pd
import matplotlib.pyplot as plt 
import matplotlib.patches as patches
import numpy as np
import math
import yaml
import os

from scipy.spatial import ConvexHull
import shapely as sp
from shapely.geometry import Polygon, Point, MultiPolygon
from shapely.ops import unary_union
from concave_hull import concave_hull

from disjoint import build_disjoint_sets # in data/scripts
from process_csv import obstacle_map_dir

def generate_test_data(output_filename):
    width, height = 20, 20
    data = []
    for y in range(height):
        for x in range(width):
            k = -((x - width / 2)**2) + 100
            data.append((x, y, k))
    df = pd.DataFrame(data, columns=['x', 'y', 'k'])
    df.to_csv(output_filename, index=False)

# load data into 2d array, assuming terrain map format is x,y,k
def df_to_grid(df):
    # Get sorted unique x and y coordinates
    x_unique = np.sort(df['x'].unique())
    y_unique = np.sort(df['y'].unique())

    grid = np.empty((len(y_unique), len(x_unique)))
    grid[:] = np.nan  # initialize with NaN for missing data

    # Create a mapping from coordinate to index
    x_idx_map = {v: i for i, v in enumerate(x_unique)}
    y_idx_map = {v: i for i, v in enumerate(y_unique)}

    # Fill grid with stiffness values
    for _, row in df.iterrows():
        xi = x_idx_map[row['x']]
        yi = y_idx_map[row['y']]
        grid[yi, xi] = row[2]

    return grid, y_unique, x_unique

def select_evenly_spaced_ticks(coords, num_ticks=10):
    n = len(coords)
    if n <= num_ticks:
        indices = np.arange(n)
        # If fewer points than desired ticks, just use all
        return coords, coords
    else:
        indices = np.linspace(0, n - 1, num_ticks, dtype=int)
        tick_values = np.array(coords)[indices]
        return indices, tick_values

"""
Obstacle extraction functions from mapping_package/utils
"""
def get_unsafe_points_from_map(Y, x_coords, y_coords, threshold):
    """
    Args:
        Y: flattened value (environment) map
        x_coords: x position of each value in Y
        y_coords: y position of each value in Y
        threshold: values below this are considered 'unsafe'
    
    Assume Y, x_coords, and y_coords have the same shape.
        
    Returns:
        position_set_filtered: Array of safe (x, y) locations
    """

    # Set of safe array indices
    Y = np.array(Y)
    unsafe_indices = np.where(Y < threshold)[0]

    # Set of safe real-world coordinates
    position_set_filtered = [(x_coords[i], y_coords[i]) for i in unsafe_indices]
   
    return position_set_filtered

def get_obstacles(Y, x_coords, y_coords, threshold):
    """
    Create obstacle polygons from filtered safe positions.
    
    Args:
        position_set_filtered: Array of safe positions
        
    Returns:
        polygon_list: List of obstacle polygons
    """

    position_set_filtered = get_unsafe_points_from_map(Y, x_coords, y_coords, threshold)

    if not position_set_filtered:
        print("[WARNING] No unsafe points found below the threshold.")
        return []  # Return an empty list of polygons

    # Build disjoint sets from the position set
    disjoint_sets = build_disjoint_sets(position_set_filtered, threshold=1)

    polygon_list = []
    
    # Create polygons from the disjoint sets
    for i, group in enumerate(disjoint_sets.subsets()):
        if len(group) > 2:
            points = [position_set_filtered[i] for i in group]
            points = np.array(points)
            # Check if points are collinear
            if np.all(points[:, 0] == points[0, 0]):
                print('points are collinear')
                continue
            
            # Create concave hull for larger groups
            if len(group) > 3:
                hull = concave_hull(points, concavity=1, length_threshold=0)

                # Add all polygons if alphashape created multiple from the cluster
                if isinstance(hull, MultiPolygon):
                    for poly in hull:
                        polygon_list.append(poly)
                    continue
                else:
                    poly = Polygon(hull)
            else:
                poly = Polygon(points) 
            
            # default is CW points, so reverse to get CCW points
            #reversed_coords = (list(poly.exterior.coords))[::-1]

            poly = Polygon(list(poly.exterior.coords))
            polygon_list.append(poly)
    
    return polygon_list

def load_yaml(file_path):
    with open(file_path, 'r') as f:
        return yaml.safe_load(f)

def remove_collinear_points(poly):
    # Convert [x1,...], [y1,...] format to [(x1,y1), ...]
    coords = list(poly.exterior.coords)
    new_coords = []

    for i in range(0, len(coords)):
        # keep first and last coord.
        if i == 0 or i == (len(coords) - 1):
            new_coords.append(coords[i])
            continue

        prev = coords[i - 1]
        curr = coords[i]
        next = coords[(i + 1) % len(coords)] # loop around

        # Compute area of triangle formed by prev, curr, next
        # If area is near zero, the point is collinear
        area = abs(
            (prev[0] * (curr[1] - next[1]) +
             curr[0] * (-prev[1] + next[1]) +
             next[0] * (prev[1] - curr[1])) / 2.0
        )

        tolerance = 0.01
        if area > tolerance:
            new_coords.append(curr)

    return Polygon(new_coords)

def plot_heatmap(df, polygon_list):
    # terrain map to 2D array
    data, x_coords, y_coords = df_to_grid(df)
    print(f'heatmap dimensions: {len(data)} x {len(data[0])}')

    fig, ax = plt.subplots()

    # plot obstacles over heatmap
    heatmap = ax.imshow(data, cmap='viridis', origin='lower',
        extent=[x_coords.min(), x_coords.max(), y_coords.min(), y_coords.max()])

    # get n ticks for x and y axes
    x_tick_indices, x_tick_positions = select_evenly_spaced_ticks(x_coords, 10)
    y_tick_indices, y_tick_positions = select_evenly_spaced_ticks(y_coords, 10)

    # label ticks on axes using coord values
    ax.set_xticks(x_tick_positions)
    ax.set_yticks(y_tick_positions)

    # format labels
    ax.set_xticklabels([f"{pos:.1f}" for pos in y_tick_positions])
    ax.set_yticklabels([f"{pos:.1f}" for pos in x_tick_positions])

    for poly in polygon_list:
        print('old count: ', len(poly.exterior.xy[0]))
        poly = remove_collinear_points(poly)
        print('new count: ', len(poly.exterior.xy[0]))

        # Simplify obstacle shapes
        x, y = poly.exterior.xy

        patch = patches.Polygon(np.column_stack((x, y)),
            facecolor='none',
            edgecolor='red',
            linewidth=4
        )

        ax.add_patch(patch)

    plt.title("Stiffness Map with Extracted Obstacles")
    ax.set_xlabel('x-coordinate (m)')
    ax.set_ylabel('y-coordinate (m)')
    cbar = plt.colorbar(heatmap)
    cbar.set_label('Terrain Stiffness Value')
    plt.show()

def save_obstacle_csv(df, polygon_list, out_filename):
    # obstacle map bounding box (workspace)
    min_x = df['x'].min()
    min_y = df['y'].min()
    max_x = df['x'].max()
    max_y = df['y'].max()

    workspace = Polygon([
        (min_x, min_y),
        (max_x, min_y),
        (max_x, max_y),
        (min_x, max_y)
    ])
    polygon_list.insert(0, workspace)

    # convert shapely polygons to obstacle map CSV format
    x_coords, y_coords = [], []
    for poly in polygon_list:
        # Simplify obstacle shapes to remove collinear points
        poly = remove_collinear_points(poly)
        coords = list(poly.exterior.coords)
        print('saving obstacle CSV with vertex count: ', len(coords))
        for coord in coords:
            print(coord)

        # first and last points are duplicates, so remove last one.
        coords = coords[:-1]

        # select n evenly spaced coords.
        print('old length of obstacle CSV polygon coords: ', len(coords))
        #_, coords = select_evenly_spaced_ticks(coords, 100)
        print('new length of obstacle CSV polygon coords: ', len(coords))

        for coord in coords:
            x_coords.append(coord[0])
            y_coords.append(coord[1])
        x_coords.append("NaN")
        y_coords.append("NaN")

    # remove last NaN
    x_coords = x_coords[:-1]
    y_coords = y_coords[:-1]

    poly_filename = os.path.join(obstacle_map_dir(), out_filename)
    poly_df = pd.DataFrame([x_coords, y_coords])
    poly_df.to_csv(poly_filename, index=False, header=False)

def main():
    # read parameters from YAML file
    config = load_yaml("../../config/launch_args.yaml")
    risk_map_config = config.get('risk_map_config', {})

    risk_filename = risk_map_config.get('risk_in_file')
    obstacle_filename = risk_map_config.get('obstacle_out_file')

    # (optional) generate fake risk map data
    # generate_test_data(obstacle_filename)

    # read risk map and extract obstacles
    input_filename = os.path.join("../risk_maps", risk_filename)
    df = pd.read_csv(input_filename)
    df = df.round(0).astype(int)
    
    # Find mean stiffness
    threshold = (df.iloc[:,2].mean() + df.iloc[:,2].min()) / 2

    polygon_list = get_obstacles(df.iloc[:,2], df['x'], df['y'], threshold)

    save_obstacle_csv(df, polygon_list, obstacle_filename)

    # plot polygons over colorized risk map
    plot_heatmap(df, polygon_list)
    

if __name__ == "__main__":
    main()