#!/usr/bin/env python3
"""
ROS2 노드: 경로 계획을 받아서 실제 로봇을 움직이는 노드
"""
import json
import time
import math
from pathlib import Path
from typing import List, Dict, Any

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# 로컬 PC 전용 모듈들 (로봇 PC에는 없음 - optional import)
# 곡선 최적화 경로만 사용 (직선 로직 제거됨)
try:
    from api_to_position_curve import get_robot_path_from_addresses as get_robot_path_from_addresses_curve  # noqa: F401
    from api_to_position_curve import get_robot_path_from_coordinates  # noqa: F401
    from test_naver_map_crawl import convert_json_to_robot_commands  # noqa: F401
    LOCAL_MODULES_AVAILABLE = True
except ImportError:
    # 로봇 PC에서는 이 모듈들이 없으므로 JSON 파일로만 작동
    LOCAL_MODULES_AVAILABLE = False
    get_robot_path_from_addresses_curve = None
    get_robot_path_from_coordinates = None
    convert_json_to_robot_commands = None

SETTINGS_PATH = Path(__file__).resolve().parent / "motion_settings.json"

# ---------------------------------------------------------------------------
# Precomputed path datasets (e.g., move_path_9)
# ---------------------------------------------------------------------------
PRECOMPUTED_PATHS: Dict[str, Dict[str, Any]] = {
    # move_path_9는 --commands-json으로 전달받음
}


def load_motion_settings():
    defaults = {
        "mode": "scale",
        "scale_width": 2.0,
        "scale_height": 3.0,
        "scale_speed": 0.1,
        "real_speed": 0.2,
    }
    if SETTINGS_PATH.exists():
        try:
            with open(SETTINGS_PATH, "r", encoding="utf-8") as f:
                data = json.load(f)
            for key in defaults:
                if key in data:
                    if isinstance(defaults[key], float):
                        defaults[key] = float(data.get(key, defaults[key]))
                    else:
                        defaults[key] = data.get(key, defaults[key])
        except Exception:
            pass
    return defaults


class RobotMover(Node):
    def __init__(self):
        super().__init__('robot_mover')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('Robot Mover 노드가 시작되었습니다.')
        self.motion_settings = load_motion_settings()
        self.default_linear_speed = (
            self.motion_settings["scale_speed"]
            if self.motion_settings.get("mode") == "scale"
            else self.motion_settings["real_speed"]
        )
        
        # Publisher 연결 대기 (첫 메시지 누락 방지)
        self.get_logger().info('Publisher 연결 대기 중...')
        time.sleep(1.0)  # 1초 대기
        
        # 더미 메시지 발행 (연결 확인)
        dummy_twist = Twist()
        for _ in range(5):
            self.publisher.publish(dummy_twist)
            time.sleep(0.1)
        self.get_logger().info('Publisher 연결 완료!')

    def move_straight(self, distance_m: float, speed_mps: float = None):
        """직진 이동 (m)"""
        if distance_m <= 0:
            return
        
        speed = speed_mps if speed_mps is not None else self.default_linear_speed
        if speed <= 0:
            speed = 0.1

        self.get_logger().info(f'직진 시작: {distance_m:.3f}m (속도: {speed:.2f}m/s)')
        
        # 명령 시작 전 정지 명령 먼저 발행 (안전)
        stop_twist = Twist()
        for _ in range(3):
            self.publisher.publish(stop_twist)
            time.sleep(0.05)
        
        twist = Twist()
        twist.linear.x = speed
        
        duration = distance_m / speed
        start_time = time.time()
        
        # 루프 주기를 0.05초로 줄여서 더 자주 발행
        while (time.time() - start_time) < duration:
            self.publisher.publish(twist)
            time.sleep(0.05)
        
        # 정지 (여러 번 발행하여 확실히 정지)
        twist.linear.x = 0.0
        for _ in range(5):
            self.publisher.publish(twist)
            time.sleep(0.05)
        
        self.get_logger().info(f'직진 완료: {distance_m:.3f}m')

    def rotate(self, angle_deg: float, angular_speed_radps: float = 0.5):
        """회전 (deg, 양수: 좌회전, 음수: 우회전)"""
        if abs(angle_deg) < 0.1:
            return
        
        self.get_logger().info(f'회전 시작: {angle_deg:.2f}° (각속도: {angular_speed_radps:.2f}rad/s)')
        
        # 명령 시작 전 정지 명령 먼저 발행 (안전)
        stop_twist = Twist()
        for _ in range(3):
            self.publisher.publish(stop_twist)
            time.sleep(0.05)
        
        twist = Twist()
        angle_rad = math.radians(angle_deg)
        twist.angular.z = angular_speed_radps if angle_deg > 0 else -angular_speed_radps
        
        duration = abs(angle_rad) / angular_speed_radps
        start_time = time.time()
        
        # 루프 주기를 0.05초로 줄여서 더 자주 발행
        while (time.time() - start_time) < duration:
            self.publisher.publish(twist)
            time.sleep(0.05)
        
        # 정지 (여러 번 발행하여 확실히 정지)
        twist.angular.z = 0.0
        for _ in range(5):
            self.publisher.publish(twist)
            time.sleep(0.05)
        
        self.get_logger().info(f'회전 완료: {angle_deg:.2f}°')

    def move_curve(self, distance_m: float, linear_speed: float, angular_speed: float):
        """곡선 주행 (m)"""
        if distance_m <= 0 or linear_speed <= 0:
            return
        
        self.get_logger().info(f'곡선 시작: {distance_m:.3f}m (선속도: {linear_speed:.2f}m/s, 각속도: {angular_speed:.2f}rad/s)')
        
        # 명령 시작 전 정지 명령 먼저 발행
        stop_twist = Twist()
        for _ in range(3):
            self.publisher.publish(stop_twist)
            time.sleep(0.05)
        
        twist = Twist()
        twist.linear.x = linear_speed
        twist.angular.z = angular_speed
        
        duration = distance_m / linear_speed
        start_time = time.time()
        
        while (time.time() - start_time) < duration:
            self.publisher.publish(twist)
            time.sleep(0.05)
        
        # 정지 (여러 번 발행)
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        for _ in range(5):
            self.publisher.publish(twist)
            time.sleep(0.05)
        
        self.get_logger().info(f'곡선 완료: {distance_m:.3f}m')

    def execute_commands(self, commands: List[Dict[str, Any]]):
        self.get_logger().info(f'총 {len(commands)}개 세그먼트 실행 시작')
        
        # 실행 전 완전 정지 확인 (토픽 누락 방지)
        self.get_logger().info('명령 실행 전 정지 상태 확인 중...')
        stop_twist = Twist()
        for _ in range(10):
            self.publisher.publish(stop_twist)
            time.sleep(0.1)
        self.get_logger().info('정지 상태 확인 완료, 명령 실행 시작!')
        
        for i, cmd in enumerate(commands):
            self.get_logger().info(f'\n--- 세그먼트 {i+1}/{len(commands)}: {cmd["instruction"]} ---')
            
            # 각 명령 전에 짧은 대기
            time.sleep(0.3)
            
            cmd_type = cmd.get("type", "straight")
            
            # 스무스 커브 처리
            if cmd_type == "smooth_curve" and abs(cmd.get("angular", 0)) > 1e-3:
                self.move_curve(
                    cmd.get("distance", 0),
                    cmd.get("linear", self.default_linear_speed),
                    cmd.get("angular", 0.0)
                )
                time.sleep(0.5)
                continue
            
            # 일반 회전 + 직진
            if abs(cmd.get("turn_angle", 0)) > 0.1:
                self.rotate(cmd["turn_angle"])
                time.sleep(0.5)
            
            if cmd.get("distance", 0) > 0:
                self.move_straight(cmd["distance"], cmd.get("linear", self.default_linear_speed))
                time.sleep(0.5)
        
        self.get_logger().info('\n✅ 모든 세그먼트 실행 완료!')

    def execute_path(self, start_address: str, goal_address: str,
                     target_width_m: float = None, target_height_m: float = None,
                     path_type: str = "curve"):
        """API 기반 경로를 생성해 실행
        
        Args:
            start_address: 출발지 주소
            goal_address: 도착지 주소
            target_width_m: 목표 가로 크기 (m)
            target_height_m: 목표 세로 크기 (m)
            path_type: 경로 타입 (곡선으로 고정, 직선 로직 제거됨)
        """
        self.get_logger().info(f'경로 계획 생성 중... (곡선 최적화 경로)')
        if target_width_m is None:
            target_width_m = self.motion_settings.get("scale_width", 2.0)
        if target_height_m is None:
            target_height_m = self.motion_settings.get("scale_height", 3.0)

        try:
            # 곡선 최적화 경로만 사용
            result = get_robot_path_from_addresses_curve(
                start_address=start_address,
                goal_address=goal_address,
                target_width_m=target_width_m,
                target_height_m=target_height_m,
                angle_threshold=20,
            )
            commands = result["commands"]
            self.execute_commands(commands)
        except Exception as e:
            self.get_logger().error(f'경로 실행 중 오류: {str(e)}')
            raise

    def execute_precomputed_path(self, path_name: str):
        dataset = PRECOMPUTED_PATHS.get(path_name)
        if not dataset:
            raise ValueError(f"Precomputed path '{path_name}'을 찾을 수 없습니다.")
        commands = dataset.get("commands", [])
        self.get_logger().info(f"[사전 정의 경로] {path_name} 실행 시작")
        self.execute_commands(commands)

    def execute_path_from_json(self, json_path: str, 
                               target_width_m: float = None, target_height_m: float = None):
        """JSON 파일에서 경로 계획을 읽어서 실행 (도보 경로용)"""
        self.get_logger().info('JSON 파일에서 경로 계획 읽는 중...')
        if target_width_m is None:
            target_width_m = self.motion_settings.get("scale_width", 2.0)
        if target_height_m is None:
            target_height_m = self.motion_settings.get("scale_height", 3.0)
        
        try:
            commands = convert_json_to_robot_commands(
                json_path=json_path,
                target_width_m=target_width_m,
                target_height_m=target_height_m
            )
            self.execute_commands(commands)
        except Exception as e:
            self.get_logger().error(f'경로 실행 중 오류: {str(e)}')
            raise

    def execute_commands_from_json_file(self, json_path: str):
        """JSON 파일에서 commands를 직접 읽어서 실행"""
        self.get_logger().info(f'JSON 파일에서 commands 리스트 로드 중: {json_path}')
        try:
            with open(json_path, 'r', encoding='utf-8') as f:
                data = json.load(f)
            
            # dict 형태면 "commands" 키 추출, list면 그대로 사용
            if isinstance(data, dict):
                commands = data.get("commands")
                if not commands:
                    self.get_logger().error(f'JSON 파일에서 commands 리스트를 찾을 수 없습니다.')
                    raise ValueError("JSON 파일은 commands 리스트를 담아야 합니다.")
            elif isinstance(data, list):
                commands = data
            else:
                raise ValueError("JSON 파일은 commands 리스트 또는 {'commands': [...]} 형태여야 합니다.")
            
            if not isinstance(commands, list):
                raise ValueError("commands는 리스트여야 합니다.")
            
            self.execute_commands(commands)
        except Exception as e:
            self.get_logger().error(f'경로 실행 중 오류: {str(e)}')
            raise


def main(args=None):
    import argparse
    
    parser = argparse.ArgumentParser(description='ROS2 node to execute robot path commands')
    parser.add_argument('--json-path', type=str, help='Path to JSON file containing walking route actions')
    parser.add_argument('--commands-json', type=str, help='Path to JSON file containing commands list')
    parser.add_argument('--start-address', type=str, help='Start address for car route')
    parser.add_argument('--goal-address', type=str, help='Goal address for car route')
    parser.add_argument('--target-width', type=float, help='Target width in meters for scaling')
    parser.add_argument('--target-height', type=float, help='Target height in meters for scaling')
    parser.add_argument('--path-type', type=str, default='curve', choices=['curve'], 
                       help='Path type: "curve" (직선 로직 제거됨, 곡선으로 고정)')
    parser.add_argument('--precomputed-path', type=str, help='Use precomputed path dataset (e.g., move_path_9)')
    
    # ROS2 인자를 제외한 나머지만 파싱
    if args is None:
        import sys
        ros_args = []
        other_args = []
        for arg in sys.argv[1:]:
            if arg.startswith('__'):
                ros_args.append(arg)
            else:
                other_args.append(arg)
        parsed_args = parser.parse_args(other_args)
        rclpy.init(args=ros_args)
    else:
        # ROS2 launch에서 호출되는 경우
        ros_args = [a for a in args if a.startswith('__')]
        other_args = [a for a in args if not a.startswith('__')]
        parsed_args = parser.parse_args(other_args)
        rclpy.init(args=ros_args)
    
    mover = RobotMover()
    
    try:
        if parsed_args.commands_json:
            mover.execute_commands_from_json_file(parsed_args.commands_json)
        elif parsed_args.precomputed_path:
            mover.execute_precomputed_path(parsed_args.precomputed_path)
        elif parsed_args.json_path:
            target_width = parsed_args.target_width if parsed_args.target_width else None
            target_height = parsed_args.target_height if parsed_args.target_height else None
            mover.execute_path_from_json(parsed_args.json_path, target_width, target_height)
        elif parsed_args.start_address and parsed_args.goal_address:
            target_width = parsed_args.target_width if parsed_args.target_width else None
            target_height = parsed_args.target_height if parsed_args.target_height else None
            path_type = parsed_args.path_type if parsed_args.path_type else "curve"
            mover.execute_path(parsed_args.start_address, parsed_args.goal_address, target_width, target_height, path_type)
        else:
            mover.get_logger().error("--commands-json 또는 --precomputed-path 또는 --json-path 또는 --start-address/--goal-address를 제공해야 합니다.")
            return
    except KeyboardInterrupt:
        pass
    finally:
        mover.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

