#!/usr/bin/env python3
import rclpy
import cv2
import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from cv_bridge import CvBridge
import sensor_msgs_py.point_cloud2 as pc2
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
import tf2_geometry_msgs
import math

class DepthMapper(Node):
    def __init__(self):
        super().__init__('depth_mapper')
        
        # Map configuration
        self.map_resolution = 0.05  # 5 cm/pixel
        self.map_width = 400        # 20m
        self.map_height = 400
        self.map_origin = [-10.0, -10.0]  # [-10m, -10m] is (0,0) in grid
        self.logodds_max = 100
        self.logodds_min = -100
        
        # Sensor data storage
        self.current_depth = None
        self.current_rgb = None
        self.current_pose = None
        self.camera_info = None
        
        # Probabilistic map
        self.logodds_map = np.zeros((self.map_height, self.map_width), dtype=np.float32)
        self.global_map = np.full((self.map_height, self.map_width), -1, dtype=np.int8)
        
        # ROS2 setup
        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Subscribers
        self.create_subscription(Image, '/camera/depth/image_raw', self.depth_callback, 10)
        self.create_subscription(Image, '/camera/image_raw', self.rgb_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(CameraInfo, '/camera/camera_info', self.camera_info_callback, 10)
        
        # Publisher
        self.map_pub = self.create_publisher(
            OccupancyGrid,
            '/map',
            QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                depth=10
            )
        )
        
        # Timer for map updates
        self.create_timer(0.1, self.update_map)

    def camera_info_callback(self, msg):
        if not self.camera_info:
            self.camera_info = msg
            self.get_logger().info("Received camera intrinsics")

    def depth_callback(self, msg):
        self.current_depth = self.bridge.imgmsg_to_cv2(msg)

    def rgb_callback(self, msg):
        self.current_rgb = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def update_map(self):
        if None in [self.current_depth, self.current_rgb, self.current_pose, self.camera_info]:
            return

        try:
            # Get transform from camera to base_link
            transform = self.tf_buffer.lookup_transform(
                'base_link',
                'camera_link',  # Make sure this matches your URDF
                rclpy.time.Time()
            )
            
            # Convert depth image to 3D points (in camera frame)
            points = self.depth_to_point_cloud(self.current_depth)
            
            # Transform points to map frame
            map_points = self.transform_points_to_map(points, transform)
            
            # Update logodds map using inverse sensor model
            self.update_logodds(map_points)
            
            # Convert logodds to occupancy grid
            self.logodds_to_occupancy()
            
            # Publish map
            self.publish_map()

        except Exception as e:
            self.get_logger().error(f"TF error: {str(e)}")

    def depth_to_point_cloud(self, depth_image):
        # Convert depth image to metric point cloud using camera intrinsics
        fx = self.camera_info.k[0]
        fy = self.camera_info.k[4]
        cx = self.camera_info.k[2]
        cy = self.camera_info.k[5]
        
        rows, cols = depth_image.shape
        points = []
        
        for v in range(rows):
            for u in range(cols):
                z = depth_image[v, u]
                if z > 0 and not math.isinf(z):
                    x = (u - cx) * z / fx
                    y = (v - cy) * z / fy
                    points.append([x, y, z])
        
        return np.array(points)

    def transform_points_to_map(self, points, transform):
        # Convert points from camera frame to map frame
        map_points = []
        translation = np.array([
            self.current_pose.position.x,
            self.current_pose.position.y,
            self.current_pose.position.z
        ])
        
        # Simplified rotation (for 2D mapping)
        yaw = self.quaternion_to_yaw(self.current_pose.orientation)
        rotation = np.array([
            [np.cos(yaw), -np.sin(yaw), 0],
            [np.sin(yaw), np.cos(yaw), 0],
            [0, 0, 1]
        ])
        
        # Apply transform from camera to base_link
        points = points @ np.array([
            [transform.transform.rotation.w, -transform.transform.rotation.z, transform.transform.rotation.y],
            [transform.transform.rotation.z, transform.transform.rotation.w, -transform.transform.rotation.x],
            [-transform.transform.rotation.y, transform.transform.rotation.x, transform.transform.rotation.w]
        ]) + np.array([transform.transform.translation.x, 
                      transform.transform.translation.y,
                      transform.transform.translation.z])
        
        # Apply robot pose transform
        return (points @ rotation.T) + translation

    def quaternion_to_yaw(self, q):
        # Convert quaternion to yaw rotation (simplified for 2D)
        return math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))

    def update_logodds(self, points):
        # Inverse sensor model for occupancy mapping
        free_cells = set()
        occupied_cells = set()
        
        # Convert points to grid coordinates
        for point in points:
            if abs(point[2]) > 0.2:  # Ignore floor/ceiling
                mx = int((point[0] - self.map_origin[0]) / self.map_resolution)
                my = int((point[1] - self.map_origin[1]) / self.map_resolution)
                
                if 0 <= mx < self.map_width and 0 <= my < self.map_height:
                    occupied_cells.add((mx, my))
                    
                    # Bresenham's line algorithm for free space
                    robot_x = int((self.current_pose.position.x - self.map_origin[0]) / self.map_resolution)
                    robot_y = int((self.current_pose.position.y - self.map_origin[1]) / self.map_resolution)
                    
                    for (bx, by) in self.bresenham(robot_x, robot_y, mx, my):
                        if (bx, by) != (mx, my):
                            free_cells.add((bx, by))
        
        # Update logodds
        for (x, y) in free_cells:
            self.logodds_map[y, x] = max(self.logodds_map[y, x] - 5, self.logodds_min)
            
        for (x, y) in occupied_cells:
            self.logodds_map[y, x] = min(self.logodds_map[y, x] + 10, self.logodds_max)

    def bresenham(self, x0, y0, x1, y1):
        # Bresenham's line algorithm
        points = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        
        while True:
            points.append((x0, y0))
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy
        return points

    def logodds_to_occupancy(self):
        # Convert logodds to occupancy probabilities
        prob_map = 1.0 - 1.0 / (1.0 + np.exp(self.logodds_map))
        self.global_map = np.clip((prob_map * 100).astype(np.int8), 0, 100)
        self.global_map[self.logodds_map < -80] = -1  # Unknown areas

    def publish_map(self):
        occupancy_grid = OccupancyGrid()
        occupancy_grid.header.stamp = self.get_clock().now().to_msg()
        occupancy_grid.header.frame_id = 'map'
        
        occupancy_grid.info.resolution = self.map_resolution
        occupancy_grid.info.width = self.map_width
        occupancy_grid.info.height = self.map_height
        occupancy_grid.info.origin.position.x = self.map_origin[0]
        occupancy_grid.info.origin.position.y = self.map_origin[1]
        occupancy_grid.info.origin.orientation.w = 1.0
        
        # Flip Y-axis for RViz display
        flipped_map = np.flipud(self.global_map)
        occupancy_grid.data = flipped_map.flatten().astype(np.int8).tolist()
        self.map_pub.publish(occupancy_grid)

def main():
    rclpy.init()
    node = DepthMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
