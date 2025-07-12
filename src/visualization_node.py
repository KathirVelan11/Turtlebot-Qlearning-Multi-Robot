#!/usr/bin/env python3

import rospy
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point, Vector3
from std_srvs.srv import Empty, EmptyResponse

class QlearningVisualizer:
    def __init__(self):
        # Initialize ROS publishers
        self.grid_pub = rospy.Publisher('/qlearning/explored_cells', MarkerArray, queue_size=10)
        self.q_value_pub = rospy.Publisher('/qlearning/q_values', MarkerArray, queue_size=10)
        self.path_pub = rospy.Publisher('/qlearning/robot_paths', MarkerArray, queue_size=10)
        
        # Visualization parameters
        self.grid_size = 0.25  # Size of each grid cell
        self.marker_height = 0.05
        
        # Global sets for explored grids and robot paths
        self.global_explored_grids = set()
        self.robot_paths = {}  # Dictionary to store paths for each robot
        
        # Color schemes
        self.explored_color = ColorRGBA(0.0, 1.0, 0.0, 0.3)  # Semi-transparent green
        self.q_value_colors = [
            ColorRGBA(0.0, 0.0, 1.0, 0.7),  # Blue
            ColorRGBA(0.0, 1.0, 0.0, 0.7),  # Green
            ColorRGBA(1.0, 0.0, 0.0, 0.7)   # Red
        ]
        self.robot_colors = [
            ColorRGBA(1.0, 0.0, 0.0, 1.0),  # Red
            ColorRGBA(0.0, 1.0, 0.0, 1.0),  # Green
            ColorRGBA(0.0, 0.0, 1.0, 1.0),  # Blue
            ColorRGBA(1.0, 1.0, 0.0, 1.0)   # Yellow
        ]
        
        # Initialize ROS service to clear visualizations
        rospy.Service('/qlearning/clear_visualization', Empty, self.clear_visualization_callback)

    def publish_explored_cells(self, explored_grids=None):
        """Publish markers for global explored grid cells"""
        # First update the global set if new grids are provided
        if explored_grids is not None:
            self.global_explored_grids.update(explored_grids)
            
        # Now continue with original implementation
        marker_array = MarkerArray()
        for i, (grid_x, grid_y) in enumerate(self.global_explored_grids):
            marker = Marker()
            marker.header.frame_id = "odom"  # Use odom frame
            marker.header.stamp = rospy.Time.now()
            marker.ns = "explored_cells"
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            
            # Convert grid coordinates to world coordinates
            marker.pose.position.x = grid_x * self.grid_size
            marker.pose.position.y = grid_y * self.grid_size
            marker.pose.position.z = self.marker_height / 2
            
            marker.pose.orientation.w = 1.0
            
            marker.scale = Vector3(self.grid_size, self.grid_size, self.marker_height)
            marker.color = self.explored_color
            
            marker_array.markers.append(marker)
        
        self.grid_pub.publish(marker_array)

    def publish_q_values(self, q_table):
        """Publish markers representing Q-values for all robots"""
        marker_array = MarkerArray()
        marker_id = 0
        
        for _, row in q_table.iterrows():
            grid_x = row['grid_x']
            grid_y = row['grid_y']
            
            # Get Q-values for all actions
            q_values = [row[f'action_{i+1}'] for i in range(5)]
            max_q = max(q_values)
            
            if max_q != 0:
                # Normalize Q-value to determine color index
                normalized_q = (max_q - min(q_values)) / (max(q_values) - min(q_values) + 1e-5)
                color_idx = int(normalized_q * (len(self.q_value_colors) - 1))
                color = self.q_value_colors[color_idx]
                
                marker = Marker()
                marker.header.frame_id = "odom"
                marker.header.stamp = rospy.Time.now()
                marker.ns = "q_values"
                marker.id = marker_id
                marker.type = Marker.CUBE
                marker.action = Marker.ADD
                
                marker.pose.position.x = grid_x * self.grid_size
                marker.pose.position.y = grid_y * self.grid_size
                marker.pose.position.z = self.marker_height
                
                marker.pose.orientation.w = 1.0
                
                marker.scale = Vector3(self.grid_size * 0.8, self.grid_size * 0.8, self.marker_height * 2)
                marker.color = color
                
                marker_array.markers.append(marker)
                marker_id += 1
        
        self.q_value_pub.publish(marker_array)

    def update_robot_paths(self, robot_positions):
        """Update and publish the paths of all robots"""
        marker_array = MarkerArray()
        for idx, (robot_name, position) in enumerate(robot_positions.items()):
            # Initialize path list for the robot if not exists
            if robot_name not in self.robot_paths:
                self.robot_paths[robot_name] = []
            
            # Append the current position to the path
            self.robot_paths[robot_name].append(Point(position[0], position[1], self.marker_height))
            
            # Create a marker for the robot's path
            path_marker = Marker()
            path_marker.header.frame_id = "odom"
            path_marker.header.stamp = rospy.Time.now()
            path_marker.ns = f"{robot_name}_path"
            path_marker.id = idx
            path_marker.type = Marker.LINE_STRIP
            path_marker.action = Marker.ADD
            path_marker.pose.orientation.w = 1.0
            path_marker.scale.x = 0.02  # Line width
            path_marker.color = self.robot_colors[idx % len(self.robot_colors)]
            path_marker.points = self.robot_paths[robot_name]
            
            marker_array.markers.append(path_marker)
        
        self.path_pub.publish(marker_array)

    # Add this method to handle single robot path updates
    def update_robot_path(self, position, robot_name="robot"):
        """Update a single robot's path"""
        robot_positions = {robot_name: position}
        self.update_robot_paths(robot_positions)

    def add_explored_grids(self, explored_grids):
        """Update the global set of explored grids"""
        self.global_explored_grids.update(explored_grids)

    def clear_visualization(self):
        """Clear all visualizations"""
        # Clear explored cells and Q-values
        empty_marker_array = MarkerArray()
        clear_marker = Marker()
        clear_marker.action = Marker.DELETEALL
        empty_marker_array.markers.append(clear_marker)
        
        self.grid_pub.publish(empty_marker_array)
        self.q_value_pub.publish(empty_marker_array)
        
        # Clear robot paths
        self.robot_paths = {}
        self.path_pub.publish(empty_marker_array)

    def clear_visualization_callback(self, req):
        """Service callback to clear visualizations"""
        self.clear_visualization()
        return EmptyResponse()
