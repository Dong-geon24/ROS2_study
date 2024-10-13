import rclpy
import time
from rclpy.node import Node



def main(args=None):
    rclpy.init(args=args)

    node = Node('test1_long')

    i = 0
    max_i = 50
    while i < max_i:
        i+=1
        # time_stamp = time.time() #실시간 시계 시스템
        # print(str(i)+"hellow world!"+str(time_stamp))
        ros_time_stamp = node.get_clock().now() #ros2에서 사용하는 시간
        node.get_logger().info(str(i)+"hellow world!"+str(ros_time_stamp)) #ros2에서 권장되는 메시지 인쇄
        time.sleep(1)
    rclpy.shutdown()

    node.destroy_node()
    rclpy.shutdown()

if __name__== "__main__":
    main()