import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

class ZoneRequestNode(Node):
    def __init__(self):
        super().__init__('zone_request_node')
        self.zone_status_sub = self.create_subscription(
            String,
            'zone_status',
            self.status_callback,
            10)
        self.robot_service_client = self.create_client(Trigger,'robot_action')
        while not self.robot_service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for robot action service...')

    def status_callback(self,msg):
        if msg.data == "inside":
            self.send_request("inside")        
        elif msg.data == "outside":
            self.send_request("outside")
    
    def send_request(self,zone_status):
        request = Trigger.Request()
        future = self.robot_service_client.call_async(request)
        future.add_done_callback(lambda future: self.handle_response(future,zone_status))
    
    def handle_response(self, future, zone_status):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f"Robot action for {zone_status}: {response.message}")
            else:
                self.get_logger().info(f"Robot action for {zone_status} failed")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = ZoneRequestNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
