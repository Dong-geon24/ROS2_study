import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            Image,
            'processed_image',
            self.listener_callback,
            10)
        self.bridge = CvBridge()
        self.coordinates = []
        cv2.namedWindow("Processed Image")
        cv2.setMouseCallback("Processed Image", self.get_coordinates)

    def get_coordinates(self, event, x, y, flags, param):
            """마우스 클릭 좌표 수집 콜백 함수"""
            if event == cv2.EVENT_LBUTTONDOWN:  # 왼쪽 마우스 버튼 클릭
                if len(self.coordinates) < 4:  # 네 개의 좌표만 수집
                    self.coordinates.append((x, y))
                    print(f"Coordinate {len(self.coordinates)}: ({x}, {y})")
                
                # 네 개의 좌표가 모두 수집되면 콜백을 비활성화하여 추가 클릭을 막음
                if len(self.coordinates) == 4:
                    cv2.setMouseCallback("Processed Image", lambda *args: None)

    def listener_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        if len(self.coordinates) == 4:
            pts = np.array(self.coordinates, np.int32)
            pts = pts.reshape((-1, 1, 2))
            cv2.polylines(cv_image, [pts], isClosed=True, color=(0, 255, 0), thickness=2)

        cv2.imshow("Processed Image", cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriber()

    try:
        rclpy.spin(node)  # Blocking spin, no custom stop mechanism
    finally:
        node.destroy_node()
        rclpy.shutdown()