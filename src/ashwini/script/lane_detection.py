#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import tf2_ros
import tf2_geometry_msgs
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from geometry_msgs.msg import Point, PointStamped
from nav_msgs.msg import OccupancyGrid
from cv_bridge import CvBridge
import pyrealsense2 as rs
from sensor_msgs import point_cloud2
from std_msgs.msg import Header
import tf.transformations as tf_trans

class LaneDetectionNode:
    def __init__(self):
        rospy.init_node('lane_detection_node', anonymous=True)
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # TF2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Camera intrinsics (will be updated from camera_info)
        self.camera_intrinsics = None
        self.depth_scale = 0.001  # RealSense depth scale (mm to m)
        
        # Lane detection parameters - using same values as test script
        self.lane_width_m = rospy.get_param('~lane_width', 3.0)  # Lane width in meters
        self.max_detection_distance = rospy.get_param('~max_distance', 10.0)  # Max detection distance
        self.min_detection_distance = rospy.get_param('~min_distance', 1.0)
        
        # HSV parameters - same as test script defaults
        # Fix: Read individual HSV components and construct arrays
        white_lower_h = rospy.get_param('~white_lower_h', 0)
        white_lower_s = rospy.get_param('~white_lower_s', 0)
        white_lower_v = rospy.get_param('~white_lower_v', 200)
        white_upper_h = rospy.get_param('~white_upper_h', 180)
        white_upper_s = rospy.get_param('~white_upper_s', 30)
        white_upper_v = rospy.get_param('~white_upper_v', 255)
        
        yellow_lower_h = rospy.get_param('~yellow_lower_h', 15)
        yellow_lower_s = rospy.get_param('~yellow_lower_s', 100)
        yellow_lower_v = rospy.get_param('~yellow_lower_v', 100)
        yellow_upper_h = rospy.get_param('~yellow_upper_h', 35)
        yellow_upper_s = rospy.get_param('~yellow_upper_s', 255)
        yellow_upper_v = rospy.get_param('~yellow_upper_v', 255)
        
        # Construct HSV arrays with correct data type for OpenCV
        self.white_lower = np.array([white_lower_h, white_lower_s, white_lower_v], dtype=np.uint8)
        self.white_upper = np.array([white_upper_h, white_upper_s, white_upper_v], dtype=np.uint8)
        self.yellow_lower = np.array([yellow_lower_h, yellow_lower_s, yellow_lower_v], dtype=np.uint8)
        self.yellow_upper = np.array([yellow_upper_h, yellow_upper_s, yellow_upper_v], dtype=np.uint8)
        
        # Publishers
        self.lane_cloud_pub = rospy.Publisher('/lane_obstacles_cloud', PointCloud2, queue_size=1)
        self.lane_costmap_pub = rospy.Publisher('/lane_obstacles_grid', OccupancyGrid, queue_size=1)
        self.debug_image_pub = rospy.Publisher('/lane_debug_image', Image, queue_size=1)
        
        # Subscribers
        self.color_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.color_callback)
        self.depth_sub = rospy.Subscriber('/camera/aligned_depth_to_color/image_raw', Image, self.depth_callback)
        self.camera_info_sub = rospy.Subscriber('/camera/color/camera_info', CameraInfo, self.camera_info_callback)
        
        # Storage for synchronized data
        self.latest_color = None
        self.latest_depth = None
        self.last_process_time = rospy.Time.now()
        
        rospy.loginfo("Lane Detection Node initialized")
        rospy.loginfo(f"White HSV range: {self.white_lower} - {self.white_upper}")
        rospy.loginfo(f"Yellow HSV range: {self.yellow_lower} - {self.yellow_upper}")
    
    def camera_info_callback(self, msg):
        """Store camera intrinsics for pixel-to-3D conversion"""
        if self.camera_intrinsics is None:
            self.camera_intrinsics = rs.intrinsics()
            self.camera_intrinsics.width = msg.width
            self.camera_intrinsics.height = msg.height
            self.camera_intrinsics.ppx = msg.K[2]  # cx
            self.camera_intrinsics.ppy = msg.K[5]  # cy
            self.camera_intrinsics.fx = msg.K[0]   # fx
            self.camera_intrinsics.fy = msg.K[4]   # fy
            self.camera_intrinsics.model = rs.distortion.brown_conrady
            self.camera_intrinsics.coeffs = list(msg.D)
            rospy.loginfo("Camera intrinsics received")
    
    def color_callback(self, msg):
        """Store latest color image"""
        self.latest_color = msg
        self.process_if_ready()
    
    def depth_callback(self, msg):
        """Store latest depth image"""
        self.latest_depth = msg
        self.process_if_ready()
    
    def process_if_ready(self):
        """Process lane detection if both color and depth are available"""
        current_time = rospy.Time.now()
        
        # Throttle processing to 10 Hz
        if (current_time - self.last_process_time).to_sec() < 0.1:
            return
            
        if (self.latest_color is not None and 
            self.latest_depth is not None and 
            self.camera_intrinsics is not None):
            
            self.process_lane_detection()
            self.last_process_time = current_time
    
    def detect_lane_markings(self, color_image):
        """Detect lane markings using computer vision - same logic as test script"""
        # Convert to HSV for better color filtering
        hsv = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
        
        # Create mask for white and yellow lane markings
        white_mask = cv2.inRange(hsv, self.white_lower, self.white_upper)
        yellow_mask = cv2.inRange(hsv, self.yellow_lower, self.yellow_upper)
        
        # Combine masks
        lane_mask = cv2.bitwise_or(white_mask, yellow_mask)
        
        # Apply Gaussian blur to reduce noise
        lane_mask = cv2.GaussianBlur(lane_mask, (5, 5), 0)
        
        # Apply morphological operations to clean up the mask
        kernel = np.ones((3, 3), np.uint8)
        lane_mask = cv2.morphologyEx(lane_mask, cv2.MORPH_CLOSE, kernel)
        lane_mask = cv2.morphologyEx(lane_mask, cv2.MORPH_OPEN, kernel)
        
        # Edge detection
        edges = cv2.Canny(lane_mask, 50, 150)
        
        # Define region of interest (lower half of image)
        height, width = edges.shape
        roi_vertices = np.array([[(0, height), (0, height//2), 
                                 (width, height//2), (width, height)]], dtype=np.int32)
        
        roi_mask = np.zeros_like(edges)
        cv2.fillPoly(roi_mask, roi_vertices, 255)
        masked_edges = cv2.bitwise_and(edges, roi_mask)
        
        return white_mask, yellow_mask, lane_mask, edges, masked_edges, roi_vertices
    
    def pixel_to_3d_point(self, u, v, depth_value):
        """Convert 2D pixel coordinates to 3D point using camera intrinsics"""
        if self.camera_intrinsics is None or depth_value == 0:
            return None
        
        # Convert depth from mm to meters
        depth_m = depth_value * self.depth_scale
        
        # Use RealSense SDK to deproject pixel to 3D point
        point_3d = rs.rs2_deproject_pixel_to_point(
            self.camera_intrinsics, [u, v], depth_m)
        
        return point_3d
    
    def transform_point_to_map(self, point_camera, timestamp):
        """Transform point from camera frame to map frame"""
        try:
            # Create point in camera frame
            point_stamped = PointStamped()
            point_stamped.header.frame_id = "camera_color_optical_frame"
            point_stamped.header.stamp = timestamp
            point_stamped.point.x = point_camera[0]
            point_stamped.point.y = point_camera[1]
            point_stamped.point.z = point_camera[2]
            
            # Transform to map frame
            transform = self.tf_buffer.lookup_transform(
                "map", "camera_color_optical_frame", timestamp, rospy.Duration(0.1))
            
            point_map = tf2_geometry_msgs.do_transform_point(point_stamped, transform)
            
            return [point_map.point.x, point_map.point.y, point_map.point.z]
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, 
                tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"Transform failed: {e}")
            return None
    
    def process_lane_detection(self):
        """Main processing function for lane detection"""
        try:
            # Convert ROS images to OpenCV format
            color_image = self.bridge.imgmsg_to_cv2(self.latest_color, "bgr8")
            depth_image = self.bridge.imgmsg_to_cv2(self.latest_depth, "16UC1")
            
            # Detect lane markings using same logic as test script
            white_mask, yellow_mask, lane_mask, edges, masked_edges, roi_vertices = self.detect_lane_markings(color_image)
            
            # Find lane pixels
            lane_pixels = np.where(masked_edges > 0)
            
            # Convert lane pixels to 3D points
            lane_points_3d = []
            valid_pixels = []
            
            for i in range(len(lane_pixels[0])):
                v = lane_pixels[0][i]  # row (y)
                u = lane_pixels[1][i]  # col (x)
                
                # Get depth value
                depth_value = depth_image[v, u]
                
                if depth_value > 0:
                    # Convert to 3D point in camera frame
                    point_3d = self.pixel_to_3d_point(u, v, depth_value)
                    
                    if point_3d is not None:
                        # Filter by distance
                        distance = np.sqrt(point_3d[0]**2 + point_3d[1]**2 + point_3d[2]**2)
                        
                        if (self.min_detection_distance < distance < self.max_detection_distance and
                            point_3d[2] > 0):  # Point should be in front of camera
                            
                            # Transform to map frame
                            point_map = self.transform_point_to_map(
                                point_3d, self.latest_color.header.stamp)
                            
                            if point_map is not None:
                                lane_points_3d.append(point_map)
                                valid_pixels.append((u, v))
            
            # Publish results
            if lane_points_3d:
                self.publish_lane_obstacles(lane_points_3d, self.latest_color.header.stamp)
                
            # Publish debug image
            self.publish_debug_image(color_image, white_mask, yellow_mask, lane_mask, valid_pixels)
            
            # Log detection stats
            rospy.logdebug(f"Detected {len(lane_points_3d)} lane points, "
                          f"White pixels: {np.sum(white_mask > 0)}, "
                          f"Yellow pixels: {np.sum(yellow_mask > 0)}, "
                          f"Total lane pixels: {np.sum(lane_mask > 0)}")
            
        except Exception as e:
            rospy.logerr(f"Error in lane detection processing: {e}")
    
    def publish_lane_obstacles(self, lane_points, timestamp):
        """Publish lane points as obstacles for navigation"""
        # Create point cloud
        header = Header()
        header.stamp = timestamp
        header.frame_id = "map"
        
        # Convert points to point cloud format
        points = []
        for point in lane_points:
            points.append([point[0], point[1], point[2]])
        
        # Create point cloud message
        cloud_msg = point_cloud2.create_cloud_xyz32(header, points)
        self.lane_cloud_pub.publish(cloud_msg)
        
        # Create occupancy grid for costmap
        self.create_lane_costmap(lane_points, timestamp)
    
    def create_lane_costmap(self, lane_points, timestamp):
        """Create occupancy grid from lane points for costmap integration"""
        if not lane_points:
            return
        
        # Grid parameters
        resolution = 0.1  # 10cm per cell
        width = 200  # 20m x 20m grid
        height = 200
        
        # Create occupancy grid
        grid = OccupancyGrid()
        grid.header.stamp = timestamp
        grid.header.frame_id = "map"
        grid.info.resolution = resolution
        grid.info.width = width
        grid.info.height = height
        
        # Set origin (center the grid around robot)
        try:
            transform = self.tf_buffer.lookup_transform(
                "map", "base_link", timestamp, rospy.Duration(0.1))
            
            robot_x = transform.transform.translation.x
            robot_y = transform.transform.translation.y
            
            grid.info.origin.position.x = robot_x - (width * resolution) / 2
            grid.info.origin.position.y = robot_y - (height * resolution) / 2
            grid.info.origin.position.z = 0
            
        except:
            grid.info.origin.position.x = -10
            grid.info.origin.position.y = -10
            grid.info.origin.position.z = 0
        
        # Initialize grid with unknown values
        grid.data = [-1] * (width * height)
        
        # Mark lane points as obstacles
        for point in lane_points:
            # Convert world coordinates to grid coordinates
            grid_x = int((point[0] - grid.info.origin.position.x) / resolution)
            grid_y = int((point[1] - grid.info.origin.position.y) / resolution)
            
            # Check bounds
            if 0 <= grid_x < width and 0 <= grid_y < height:
                index = grid_y * width + grid_x
                grid.data[index] = 100  # Mark as obstacle
        
        # Publish the costmap
        self.lane_costmap_pub.publish(grid)
    
    def publish_debug_image(self, color_image, white_mask, yellow_mask, lane_mask, valid_pixels):
        """Publish debug visualization image"""
        debug_image = color_image.copy()
        
        # Create overlay images
        overlay = debug_image.copy()
        
        # Show white mask in blue
        white_colored = np.zeros_like(debug_image)
        white_colored[:, :, 0] = white_mask  # Blue channel
        overlay = cv2.addWeighted(overlay, 0.8, white_colored, 0.4, 0)
        
        # Show yellow mask in yellow
        yellow_colored = np.zeros_like(debug_image)
        yellow_colored[:, :, 1] = yellow_mask  # Green channel
        yellow_colored[:, :, 2] = yellow_mask  # Red channel (Green + Red = Yellow)
        overlay = cv2.addWeighted(overlay, 0.8, yellow_colored, 0.4, 0)
        
        # Mark valid 3D points in green
        for u, v in valid_pixels:
            cv2.circle(overlay, (u, v), 2, (0, 255, 0), -1)
        
        # Add text information
        y_offset = 30
        cv2.putText(overlay, f"LANE DETECTION - ROS NODE", 
                   (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        y_offset += 30
        cv2.putText(overlay, f"White pixels: {np.sum(white_mask > 0)}", 
                   (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        y_offset += 25
        cv2.putText(overlay, f"Yellow pixels: {np.sum(yellow_mask > 0)}", 
                   (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        y_offset += 25
        cv2.putText(overlay, f"Total lane pixels: {np.sum(lane_mask > 0)}", 
                   (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        y_offset += 25
        cv2.putText(overlay, f"3D points: {len(valid_pixels)}", 
                   (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        # Add parameter info
        y_offset += 40
        cv2.putText(overlay, f"White HSV: {self.white_lower} - {self.white_upper}", 
                   (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
        y_offset += 20
        cv2.putText(overlay, f"Yellow HSV: {self.yellow_lower} - {self.yellow_upper}", 
                   (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
        
        # Convert back to ROS message
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(overlay, "bgr8")
            debug_msg.header = self.latest_color.header
            self.debug_image_pub.publish(debug_msg)
        except Exception as e:
            rospy.logerr(f"Error publishing debug image: {e}")

if __name__ == '__main__':
    try:
        node = LaneDetectionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass