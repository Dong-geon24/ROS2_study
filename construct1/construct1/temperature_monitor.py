# mars_rover_systems 패키지라는 이름의 새 스크립트, temperature_monitor.py 라는 이름의 새 스크립트를 만듭니다.
# entry point 이름은 temperaturemonitor_executable 이어야 합니다.
# 사용자 정의 ROS2 노드 클래스를 생성합니다.
# main() 메서드 이름을 사용하는 대신 start_monitor() 을 사용하여 모든 노드 초기화와 spin() 을 포함합니다.
# heartbeat_executable 및 temperaturemonitor_executable 의 진입점을 시작하는 새 launch 파일을 생성하여 심장 박동 시스템과 온도 모니터 시스템을 모두 시작합니다. 이름은 start_mars_rover_systems.launch.py 이 되어야 합니다.
# Python 모듈 메서드 random.uniform(20.0, 100.0) 를 사용하여 온도 변동을 시뮬레이션해야 합니다.
# 화성 탐사선의 온도가 매초마다 화면에 표시되어야 합니다.
# 온도가 고온으로 간주되는 임계값 70.0 ºC 을 초과하는 경우 경고 메시지를 발행해야 합니다.
# ROS2의경고 메시지는 info 을 변경하여 발행됩니다:

import rclpy
from rclpy.node import Node
import random

class TemperatureMonitorNode(Node):
    def __init__(self):
        self.temperature_threshhold = 70.0
        super().__init__('temperature')

        self.create_timer(1.0,self.timer_callback)
    
    def get_temperature(self):
        temperature = random.uniform(20.0, 100.0)
        return temperature

    def timer_callback(self):
        # ros_time_stamp = self.get_clock().now()
        current_temperature = self.get_temperature()
        self.get_logger().info(f"Current Temperature : {current_temperature}")
        if current_temperature > self.temperature_threshhold: 
            self.get_logger().warn(f"High Temperature detected! :  {current_temperature}")

def main(args=None):
    rclpy.init()
    node = TemperatureMonitorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()