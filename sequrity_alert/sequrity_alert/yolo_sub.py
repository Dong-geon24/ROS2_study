import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
from shapely.geometry import Polygon, Point as ShapelyPoint

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.image_subscription = self.create_subscription(
            Image,
            'processed_image',
            self.listener_callback,
            10)
        self.coord_subscription = self.create_subscription(
            Point,
            'car_coordinates',
            self.coord_callback,10)
        self.status_publisher = self.create_publisher(String,'zone_status',10)
        self.bridge = CvBridge()
        self.coordinates = [] #폴리곤 좌표 리스트
        self.car_center = None #차량 중심좌표 저장 변수


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
        status_msg = String()

        if len(self.coordinates) == 4:
            pts = np.array(self.coordinates, np.int32)
            pts = pts.reshape((-1, 1, 2))
            cv2.polylines(cv_image, [pts], isClosed=True, color=(0, 255, 0), thickness=2)
            polygon = Polygon(self.coordinates)

            if self.car_center:
                center_x, center_y = int(self.car_center[0]),int(self.car_center[1])
                car_point = ShapelyPoint(center_x,center_y)
                cv2.circle(cv_image,(center_x,center_y), 5, (0, 0, 255), -1) 
                if polygon.contains(car_point):
                    #데이터 송신부
                    status_msg.data = "inside"
                    self.status_publisher.publish(status_msg)
                    #터미널 - 화면에 출력부
                    self.get_logger().info("car_inside")
                    cv2.putText(cv_image, "car_inside",(10,30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
                else:
                    status_msg.data = "outside"
                    self.status_publisher.publish(status_msg)
                    self.get_logger().info("car_outside")
                    cv2.putText(cv_image, "car_outside",(10,30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        cv2.imshow("Processed Image", cv_image)
        cv2.waitKey(1)

    def coord_callback(self, msg):
        self.car_center = (msg.x, msg.y)
        print(f"Received car center coordinates: ({msg.x}, {msg.y})")

        
def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriber()

    try:
        rclpy.spin(node)  # Blocking spin, no custom stop mechanism
    finally:
        node.destroy_node()
        rclpy.shutdown()