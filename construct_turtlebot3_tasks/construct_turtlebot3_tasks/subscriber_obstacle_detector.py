#! /usr/bin/env python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile

class ObstacleDetectorNode(Node):
    def __init__(self, node_name='obstacle_detector_node'):
        self.node_name = node_name
        super().__init__(self.node_name)
        self.get_logger().info(self.node_name + " Ready...")

        self.subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
    
    def scan_callback(self, msg):
        # min_distance = min(msg.ranges)
        # self.get_logger().info(f'Minimum distance: {min_distance:.2f}m')
        sectors = {
            "front":(0,180),
            "back": (181,359)
        }
        min_distances = {key: float('inf') for key in sectors.keys()}

        for sector, (start_idx,end_idx) in sectors.items():
            if start_idx < len(msg.ranges) and end_idx < len(msg.ranges):
                sector_ranges = msg.ranges[start_idx:end_idx + 1]
                if sector_ranges:
                    min_distances[sector] = min(sector_ranges)
        
        for sector, min_distance in min_distances.items():
            self.get_logger().info(f'{sector}: {min_distance:.2f} meters')

        obstacle_threshold = 0.8
        detections = {sector: min_distance < obstacle_threshold for sector,min_distance in min_distances.items()}

        if detections['front']:
            arbitrary_direction = 'right'
            action = 'Selected Turn Arbitrary Direction ' + arbitrary_direction
        elif detections['back']:
            arbitrary_direction = 'left'
            action = 'Selected Turn Arbitrary Direction ' + arbitrary_direction
        else: 
            action = 'Go Forwards'                   

        self.get_logger().info(f'Suggested action: {action}' )     
        
def main(args=None):
    rclpy.init()
    node = ObstacleDetectorNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=="__main__":
    main()