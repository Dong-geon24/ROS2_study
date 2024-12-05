import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import numpy as np
import threading
import math


class YoloProcessor(Node):
    def __init__(self):
        super().__init__('yolo_processor')

        # Subscribers and Publishers
        self.image_subscriber = self.create_subscription(Image, '/skidbot/camera_sensor/image_raw', self.image_callback, 10)
        self.image_publisher_ = self.create_publisher(Image, '/processed_img', 10)
        self.coord_publisher_ = self.create_publisher(Point, '/car_coordinates', 10)

        # YOLO Model and Helper Initialization
        self.bridge = CvBridge()
        self.model = YOLO('/home/road2022/Documents/data/rokey_on_site/best.pt')
        self.processed_frame = None
        self.classNames = ['car']

        # Processing control
        self.lock = threading.Lock()
        self.current_frame = None

    def image_callback(self, msg):
        """Callback to process received images."""
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # Run YOLO detection
            results = self.model(cv_image, stream=True)
            for r in results:
                for box in r.boxes:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    confidence = math.ceil(box.conf[0] * 100) / 100
                    cls = int(box.cls[0])

                    # Draw bounding boxes and labels
                    cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 0, 255), 2)
                    cv2.putText(cv_image, f"{self.classNames[cls]}: {confidence}", (x1, y1 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

                    # Publish car coordinates if the detected object is a car
                    if self.classNames[cls] == "car":
                        car_x = (x1 + x2) / 2
                        car_y = (y1 + y2) / 2
                        car_position = Point(x=car_x, y=car_y, z=0.0)
                        self.coord_publisher_.publish(car_position)
                        self.get_logger().info(f"Car center coordinates: ({car_x}, {car_y})")

            # Publish processed image
            ros_image = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
            self.image_publisher_.publish(ros_image)

        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

    def destroy_node(self):
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = YoloProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
