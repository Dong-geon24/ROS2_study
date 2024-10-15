import rclpy
from rclpy.node import Node
from custom_interfaces.msg import RoverEvents
from std_msgs.msg import String
from geometry_msgs.msg import Pose

def main(args=None):
    rclpy.init(args=args)
    node = Node('test_node')
    
    # RoverEvents 메시지 생성
    rover_event_msg = RoverEvents()
    rover_event_msg.info = String()
    rover_event_msg.info.data = "Test message"  # 메시지 안의 info 필드에 문자열 할당
    rover_event_msg.rover_location = Pose()  # rover_location 필드에 Pose 메시지 할당
    
    node.get_logger().info(f'Rover Event Info: {rover_event_msg.info.data}')
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
