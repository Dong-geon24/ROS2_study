#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import ReliabilityPolicy, QoSProfile
import math
import tf_transformations

class AutonomousExplorationNode(Node):
    def __init__(self):
        super().__init__('autonomous_exploaration_node')

        self.subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )

        self.subscriver_odom = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )

        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.turning = False
        self.turn_direction = -0.5

        # State tracking
        self.distance_from_origin = 0.0
        self.returning_to_origin = False
        self.current_position = {'x': 0.0, 'y': 0.0}
        self.yaw = 0.0

        self.get_logger().info('Autonomous Exploration Node Ready...')

    def scan_callback(self, msg):
        if self.returning_to_origin:
            self.return_to_origin()
            return
        
        sectors = {
            "Right_Rear": (270, 359),   
            "Right": (180, 269),         
            "Front_Right": (90, 179),    
            "Front_Left": (0, 89),       
            "Left": (270, 359),          
            "Left_Rear": (180, 269)
        }
        min_distances = {key: float('inf') for key in sectors.keys()}

        for sector, (start_idx, end_idx) in sectors.items():
            if start_idx < len(msg.ranges) and end_idx < len(msg.ranges):
                sector_ranges = msg.ranges[start_idx:end_idx + 1]
                if sector_ranges:
                    min_distances[sector] = min(sector_ranges)

        obstacle_threshold = 0.8
        detections = {sector: min_distance < obstacle_threshold for sector, min_distance in min_distances.items()}

        action = Twist()
        if detections["Front_Left"] or detections["Front_Right"]:
            if not self.turning:
                # Start turning if not already turning
                self.turning = True
                self.turn_direction = -0.5  # Turning right
            action.angular.z = self.turn_direction  # Continue turning
            self.get_logger().info('Obstacle ahead, turning to clear path.')
        else:
            self.turning = False  # Stop turning when the front is clear

        # Side detections
        if detections["Left"]:
            action.linear.x = 0.2  # Move forward slowly
            action.angular.z = -0.3  # Slight right turn
            self.get_logger().info('Obstacle on the left, turning slightly right.')
        elif detections["Right"]:
            action.linear.x = 0.2  # Move forward slowly
            action.angular.z = 0.3  # Slight left turn
            self.get_logger().info('Obstacle on the right, turning slightly left.')
        # Rear detections
        elif detections["Right_Rear"]:
            action.linear.x = 0.3  # Move forward
            self.get_logger().info('Obstacle on the right rear, moving forward.')
        elif detections["Left_Rear"]:
            action.linear.x = 0.3  # Move forward
            self.get_logger().info('Obstacle on the left rear, moving forward.')
        else:
            action.linear.x = 0.5  # Move forward
            self.get_logger().info('No obstacles, moving forward.')

        self.publisher_.publish(action)

    def odom_callback(self, msg):
        # Extract the x, y coordinates from the odometry message
        self.current_position['x'] = msg.pose.pose.position.x
        self.current_position['y'] = msg.pose.pose.position.y

        # Calculate the distance from the origin (0,0)
        self.distance_from_origin = math.sqrt(self.current_position['x']**2 + self.current_position['y']**2)
        self.get_logger().info(f'Distance from origin: {self.distance_from_origin:.2f} meters')

        # Calculate the yaw (orientation around the z-axis)
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, self.yaw) = tf_transformations.euler_from_quaternion(orientation_list)

        # If the distance exceeds PERIMETER_DIST meters, initiate return to origin behavior
        PERIMETER_DIST = 5.0
        if self.distance_from_origin > PERIMETER_DIST:
            self.returning_to_origin = True
        elif self.distance_from_origin <= PERIMETER_DIST and self.returning_to_origin:
            self.returning_to_origin = False
            self.get_logger().info('Within 7 meters of origin, resuming normal operation.')

    def return_to_origin(self):
        action = Twist()

        # Calculate the desired angle to the origin
        desired_yaw = math.atan2(-self.current_position['y'], -self.current_position['x'])

        # Calculate the difference between current yaw and desired yaw
        yaw_error = desired_yaw - self.yaw

        # Normalize the yaw error to the range [-pi, pi]
        yaw_error = (yaw_error + math.pi) % (2 * math.pi) - math.pi

        # If the yaw error is significant, rotate towards the origin
        if abs(yaw_error) > 0.1:  # 0.1 radians threshold for orientation
            action.angular.z = 0.5 if yaw_error > 0 else -0.5
            self.get_logger().info(f'Turning towards origin. Yaw error: {yaw_error:.2f}')
        else:
            action.linear.x = 0.5
            self.get_logger().info('Heading towards origin.')

        self.publisher_.publish(action)

def main(args=None):
    rclpy.init(args=args)
    node = AutonomousExplorationNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
