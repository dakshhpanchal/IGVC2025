#!/usr/bin/env python3

import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
import tf2_ros
import tf2_geometry_msgs

class LaneCostmapPlugin:
    def __init__(self):
        rospy.init_node('lane_costmap_plugin', anonymous=True)
        
        # TF2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Parameters
        self.inflation_radius = rospy.get_param('~inflation_radius', 0.5)  # meters
        self.cost_scaling_factor = rospy.get_param('~cost_scaling_factor', 10.0)
        self.update_frequency = rospy.get_param('~update_frequency', 5.0)  # Hz
        
        # Publishers
        self.costmap_pub = rospy.Publisher('/move_base/local_costmap/lane_layer', OccupancyGrid, queue_size=1)
        self.global_costmap_pub = rospy.Publisher('/move_base/global_costmap/lane_layer', OccupancyGrid, queue_size=1)
        
        # Subscribers
        self.lane_cloud_sub = rospy.Subscriber('/lane_obstacles_cloud', PointCloud2, self.lane_cloud_callback)
        self.lane_grid_sub = rospy.Subscriber('/lane_obstacles_grid', OccupancyGrid, self.lane_grid_callback)
        
        # Storage
        self.latest_lane_points = []
        self.latest_lane_grid = None
        
        # Timer for periodic updates
        self.update_timer = rospy.Timer(rospy.Duration(1.0/self.update_frequency), self.update_costmaps)
        
        rospy.loginfo("Lane Costmap Plugin initialized")
    
    def lane_cloud_callback(self, msg):
        """Process incoming lane point cloud"""
        try:
            # Extract points from point cloud
            points = []
            for point in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
                points.append([point[0], point[1], point[2]])
            
            self.latest_lane_points = points
            
        except Exception as e:
            rospy.logerr(f"Error processing lane point cloud: {e}")
    
    def lane_grid_callback(self, msg):
        """Store latest lane grid"""
        self.latest_lane_grid = msg
    
    def update_costmaps(self, event):
        """Update both local and global costmaps with lane obstacles"""
        if self.latest_lane_grid is not None:
            # Publish to both local and global costmap layers
            self.costmap_pub.publish(self.latest_lane_grid)
            self.global_costmap_pub.publish(self.latest_lane_grid)
    
    def create_inflated_costmap(self, lane_points, frame_id, timestamp):
        """Create an inflated costmap from lane points"""
        if not lane_points:
            return None
        
        # Grid parameters
        resolution = 0.05  # 5cm per cell for higher resolution
        width = 400  # 20m x 20m grid
        height = 400
        
        # Create occupancy grid
        grid = OccupancyGrid()
        grid.header.stamp = timestamp
        grid.header.frame_id = frame_id
        grid.info.resolution = resolution
        grid.info.width = width
        grid.info.height = height
        
        # Set origin (center the grid around robot)
        try:
            transform = self.tf_buffer.lookup_transform(
                frame_id, "base_link", timestamp, rospy.Duration(0.1))
            
            robot_x = transform.transform.translation.x
            robot_y = transform.transform.translation.y
            
            grid.info.origin.position.x = robot_x - (width * resolution) / 2
            grid.info.origin.position.y = robot_y - (height * resolution) / 2
            grid.info.origin.position.z = 0
            
        except:
            grid.info.origin.position.x = -10
            grid.info.origin.position.y = -10
            grid.info.origin.position.z = 0
        
        # Initialize grid with free space
        grid.data = [0] * (width * height)
        
        # Mark lane points and inflate
        inflation_cells = int(self.inflation_radius / resolution)
        
        for point in lane_points:
            # Convert world coordinates to grid coordinates
            grid_x = int((point[0] - grid.info.origin.position.x) / resolution)
            grid_y = int((point[1] - grid.info.origin.position.y) / resolution)
            
            if 0 <= grid_x < width and 0 <= grid_y < height:
                # Mark the obstacle and inflate around it
                for dx in range(-inflation_cells, inflation_cells + 1):
                    for dy in range(-inflation_cells, inflation_cells + 1):
                        nx, ny = grid_x + dx, grid_y + dy
                        if 0 <= nx < width and 0 <= ny < height:
                            distance = np.sqrt(dx*dx + dy*dy) * resolution
                            
                            if distance <= self.inflation_radius:
                                index = ny * width + nx
                                
                                # Calculate cost based on distance
                                if distance == 0:
                                    cost = 100  # Lethal obstacle
                                else:
                                    # Exponential decay
                                    cost = int(100 * np.exp(-self.cost_scaling_factor * distance / self.inflation_radius))
                                    cost = max(1, min(99, cost))  # Clamp between 1-99
                                
                                # Only increase cost, don't decrease
                                current_cost = grid.data[index]
                                grid.data[index] = max(current_cost, cost)
        
        return grid

if __name__ == '__main__':
    try:
        plugin = LaneCostmapPlugin()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass 