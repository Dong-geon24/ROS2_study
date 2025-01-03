# git clone https://bitbucket.org/theconstructcore/basic_ros2_extra_files.git

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
from construct_turtlebot3_tasks.plant_detector import PlantDetector

class PlantDetectorNode(Node):
    def __init__(self):
        super().__init__('plant_detector_node')
        self.bridge = CvBridge() #ros와 CV의 이미지 형식을 맞춰줌

        path_to_model = '/home/arclab/ros2_ws/basic_ros2_extra_files/plant_detector/best_plant_detector_model.pth'
        self.plant_detector = PlantDetector(model_path=path_to_model)

        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
    
        self.subscription # prevent unused variable warning
        self.publisher_=self.create_publisher(String,'/plant_detector',10)
        #기존에 없는 토픽을 작성하면 새롭게 발행한다 !!

    def image_callback(self,msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg,"bgr8")
        prediction = self.plant_detector.predict(cv2.cvtColor(cv_image,cv2.COLOR_BGR2RGB))

        if prediction > 0.5:
            result = f"Plant detected with confidence: {prediction:.2f}"
            self.get_logger().warning(result)
        
        else:
            result = f"No Plant detected. Confidence: {1 - prediction:.2f}"
            self.get_logger().info(result)

        msg = String()
        msg.data = result
        self.publisher_.publish(msg)
    
def main(args=None):
    rclpy.init(args=args)
    plant_detector_node = PlantDetectorNode()
    rclpy.spin(plant_detector_node)

    plant_detector_node.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()