#!/usr/bin/env python3
"""
TEST 버튼용: 로봇을 30cm 앞으로 직진시키는 간단한 ROS2 노드
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time


class TestMove(Node):
    def __init__(self):
        super().__init__('test_move')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        # Publisher가 준비될 때까지 잠시 대기
        time.sleep(0.5)

    def move_forward(self, distance_m=0.3, speed_mps=0.1):
        """30cm 직진"""
        self.get_logger().info('TEST: 30cm 직진 명령 실행 중...')
        
        duration = distance_m / speed_mps  # 3초
        
        twist = Twist()
        twist.linear.x = speed_mps
        
        start_time = time.time()
        while (time.time() - start_time) < duration:
            self.publisher.publish(twist)
            time.sleep(0.05)
        
        # 정지
        self.stop()
        self.get_logger().info('TEST: 30cm 직진 완료!')

    def stop(self):
        """정지"""
        twist = Twist()
        self.publisher.publish(twist)
        time.sleep(0.1)


def main(args=None):
    rclpy.init(args=args)
    node = TestMove()
    
    # 30cm 직진 실행
    node.move_forward(0.3, 0.1)
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

