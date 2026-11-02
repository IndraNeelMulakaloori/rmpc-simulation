#!/usr/bin/env python3



import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
from visualization_msgs.msg import Marker, MarkerArray
import math
import random 
import os 
from ament_index_python.packages import get_package_share_directory
import pandas as pd

class ObstacleSpawner(Node):
    def __init__(self):
        super().__init__('obstacle_spawner')
        self.publisher_ = self.create_publisher(PoseArray, '/dynamic_obstacles', 10)


        package_share_dir = get_package_share_directory('rmpc_f1tenth')
        waypoints_path = os.path.join(package_share_dir, 'waypoints','IMS_centerline.csv')
        # self.waypoints = []
        # self.load_centerline()

        self.df = pd.read_csv(waypoints_path)
        # Strip whitespace and remove leading '#' characters from column names
        self.df.columns = self.df.columns.str.replace('#', '').str.strip()

        self.waypoints = []

        # lets get 2 wapoints and set them as obstacles i nthe centerline 
        for _ in range(2):
            idx = random.randint(1, self.df.shape[0])
            self.get_logger().info(f"idx : {idx}\n df : {self.df.loc[3]}")

            x,y = self.df.loc[idx,'x_m'],self.df.loc[idx,'y_m']
            self.waypoints.append((x,y))

        # self.obstacles = []
        # self.generate_random_obstacles(num_obstacles=3)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.t = 0.0

        self.get_logger().info(f"Wapoints : path : {waypoints_path}\n df:{self.df.head()}")
        self.get_logger().info(f"Waypoints appended : {self.waypoints}")


    def timer_callback(self):
       
          # or whatever your global frame is

        # Static Obstacles (for Step 1 testing)
        # Assuming track coordinates for IMS or Levine. 
        # You'll need to tune these (s, d) values based on your actual track.
        
        # # Obstacle 1: Static on the left side
        # obs1 = Pose()
        # obs1.position.x = 24.0  # s-coordinate
        # obs1.position.y = 26.5   # d-coordinate
        # obs1.position.z = 0.2   # radius (we'll use z for radius temporarily to pass it)
        # msg.poses.append(obs1)
        
        # # Obstacle 2: Static on the right side further down
        # obs2 = Pose()
        # obs2.position.x = 13.0  # s-coordinate
        # obs2.position.y = 38.0  # d-coordinate
        # obs2.position.z = 0.3   # radius
        # msg.poses.append(obs2)
        
        
        # --- Step 2: Dynamic Obstacles (Uncomment later) ---
        # obs3 = Pose()
        # obs3.position.x = 60.0 + 2.0 * self.t # Moving forward at 2 m/s
        # obs3.position.y = 0.8 * math.sin(self.t) # Weaving left and right
        # obs3.position.z = 0.2 # radius
        # msg.poses.append(obs3)
        pose_array = PoseArray()
        pose_array.header.frame_id = 'map'
        pose_array.header.stamp = self.get_clock().now().to_msg()

        for obs in self.waypoints:
            pose = Pose()
            pose.position.x = obs[0]
            pose.position.y = obs[1]
            pose.position.z = 0.1  # Height off ground
            pose.orientation.w = 1.0
            
            pose_array.poses.append(pose)
            
        self.publisher_.publish(pose_array)
        self.t += 0.1

def main(args=None):
    rclpy.init(args=args)
    obstacle_spawner = ObstacleSpawner()
    rclpy.spin(obstacle_spawner)
    obstacle_spawner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
