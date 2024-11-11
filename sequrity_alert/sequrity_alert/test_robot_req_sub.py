import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

class RobotService(Node):
    def __init__(self):
        super().__init__('robot_service')
        # robot_action이라는 서비스 서버 생성
        self.srv = self.create_service(Trigger, 'robot_action', self.handle_action)

    def handle_action(self, request, response):
        # 요청이 들어왔을 때 로그를 출력
        self.get_logger().info("Received a request to perform an action based on zone status.")
        
        # 응답 설정
        response.success = True
        response.message = "Action completed successfully"
        
        return response

def main(args=None):
    rclpy.init(args=args)
    node = RobotService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
