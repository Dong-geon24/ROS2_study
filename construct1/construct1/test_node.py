import rclpy
from rclpy.node import Node

class TestNode(Node):
    def __init__(self,test_name,test_word,timer_period=0.2):
        self.test_name = test_name
        self.test_word = test_word  
        super().__init__(self.test_name)

        self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        ros_time_stamp = self.get_clock().now()
        # cnt = 0
        # while True:
        #     cnt+=1
        #     self.get_logger().info(self.test_name + cnt + str(ros_time_stamp))
        self.get_logger().info(self.test_name + 'python' + str(ros_time_stamp))
        self.get_logger().info(self.test_word + '\n')        

def main(args=None):
    rclpy.init()
    node = TestNode(test_name='WTF1',test_word='HIHI',timer_period=1.0)
    rclpy.spin(node)
    rclpy.shutdown()

def main2(args=None):
    rclpy.init()
    node = TestNode(test_name='WTF2',test_word='BYEBYE',timer_period=1.0)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__== "__main__":
    main()