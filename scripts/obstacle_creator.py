import cv2
import numpy as np
import yaml
import pandas as pd
import os
import random
# Load active map metadata
map_name = "../maps/IMS/IMS"  # e.g., 'Spielberg', 'levine', etc.
yaml_path = f"{map_name}_map.yaml"
img_path = f"{map_name}_map.png"
centerline_path = f"{map_name}_centerline.csv"
with open(yaml_path, "r") as f:
  meta = yaml.safe_load(f)

print(f"Loaded map metadata from {yaml_path}:")
print(meta)
img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
res = float(meta["resolution"])
origin = meta["origin"]  # [x_origin, y_origin, yaw]
h, w = img.shape

centerline_df = pd.read_csv(centerline_path)
centerline_df.columns = centerline_df.columns.str.replace('#', '').str.strip()
print(f"Loaded centerline from {centerline_path}: {len(centerline_df)} points.\n centerline_head:\n{centerline_df.head()}")

# Pick (x, y) world coordinates from a straight section of your centerline.csv
# Choose coordinates where the car is cruising at high speed (e.g., s ~ 50m, 120m)
## @note : These coordinates are in world frame of IMS map
## to modify you can use Rviz pose estimate to get the world coordinates 
## of the obstacles you want to add
obstacle_coords = [
  (0.190728, -10.2845),
  (26.9882,70.4085),
(52.2316, 9.98813)  # Obstacle 1 (World X, World Y in meters)
] 



for wx, wy in obstacle_coords:
  # Convert World (m) -> Map Pixel (row, col)
  px = int((wx - origin[0]) / res)
  py = int(h - 1 - (wy - origin[1]) / res)

  # Draw a solid black pillar (radius = 4 pixels = 0.20m radius)
  cv2.circle(img, (px, py), radius=4, color=0, thickness=-1)

# Save as a new map to keep the original clean
cv2.imwrite(f"{map_name}_map_obstacles.png", img)
print(f"Exported {map_name}_map_obstacles.png with {len(obstacle_coords)} obstacles.")