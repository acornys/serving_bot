#!/usr/bin/env python3
# 9/30 테스트 코드 + [추가] 모드 2: 도착 후 10초 주문 대기 → 없으면 자동 주방 복귀
import math
import threading
import time
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

# (x, y, yaw) — yaw 는 도착했을 때 바라볼 방향(라디안). 0.0 = 지도 +x 방향
WAYPOINTS = {
    'kitchen': (0.06, -0.02, 0.0),  # 주방
    '0': (0.06, -0.02, 0.0),        # 0번 (단일 지정 대기용 주방)
    '1': (1.19, 0.14, 0.0),         # 1번 테이블
    '2': (3.15, 0.62, 0.0),         # 2번 테이블
    '3': (2.38, -1.32, 0.0),        # 3번 테이블
    '4': (0.46, -1.67, 0.0),        # 4번 테이블
}

NAV2_WAIT_SEC = 5.0     # Nav2 서버를 기다리는 최대 시간
SERVE_WAIT_SEC = 3.0    # 테이블 도착 후 대기 시간 (초)
IDLE_RETURN_SEC = 10.0  # [추가] 모드 2 도착 후 추가 주문을 기다리는 시간 (초)


class DeliveryManagerNode(Node):

    def __init__(self):
        super().__init__('delivery_manager_node')

        self.cbg = ReentrantCallbackGroup()

        self._action_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose', callback_group=self.cbg
        )

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.order_sub = self.create_subscription(
            String, '/delivery_order', self.order_callback, 10, callback_group=self.cbg
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 1초 좌표 주기 출력 타이머
        self.pos_timer = self.create_timer(
            1.0, self.print_current_position_timer, callback_group=self.cbg
        )

        self.current_goal_handle = None
        self.emergency_stop = False
        self.is_delivering = False
        self.state_lock = threading.Lock()
        self.return_timer = None  # [추가] 모드 2 자동 복귀 타이머

        self.start_nav_time = 0.0
        self.last_warn_time = 0.0
        self.initial_distance_remaining = None
        self.has_moved = False
        self.target_name = ''

        self.get_logger().info('서빙 백엔드 매니저 가동 완료 (0번 주방 단일이동 지원)')

    # [추가] 자동 복귀 타이머 끄기 (새 주문 · STOP 이 오면)
    def cancel_return_timer(self):
        if self.return_timer is not None:
            self.return_timer.cancel()
            self.destroy_timer(self.return_timer)
            self.return_timer = None

    # [추가] 10초 동안 주문이 없으면 불림 → 주방 복귀
    def auto_return_callback(self):
        self.cancel_return_timer()  # 한 번만 실행
        with self.state_lock:
            if self.is_delivering:
                return
            self.is_delivering = True
        try:
            self.emergency_stop = False
            self.get_logger().info(f'{IDLE_RETURN_SEC:.0f}초 동안 추가 주문 없음 → 주방으로 자동 복귀합니다.')
            kx, ky, kyaw = WAYPOINTS['kitchen']
            if self.send_nav_goal('주방(kitchen)', kx, ky, kyaw):
                self.get_logger().info('주방 복귀 완료. 신규 주문 대기 모드로 전환.\n')
            else:
                self.get_logger().error('주방 복귀 실패. 로봇 위치를 확인하세요. (신규 주문은 받습니다)\n')
        finally:
            self.target_name = ''
            with self.state_lock:
                self.is_delivering = False

    def print_current_position_timer(self):
        """1초 주기로 로봇의 실시간 지도 좌표 출력"""
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            curr_x = t.transform.translation.x
            curr_y = t.transform.translation.y
            status_text = f'[{self.target_name} 주행 중]' if self.is_delivering else '[주문 대기 중]'
            self.get_logger().info(f'{status_text} 현재 좌표 -> X: {curr_x:.2f} m, Y: {curr_y:.2f} m')
        except TransformException:
            pass

    def stop_robot_hardware(self):
        """바퀴 모터에 속도 0을 즉시 주입하여 완전 정지"""
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        for _ in range(15):
            self.cmd_vel_pub.publish(stop_msg)
            time.sleep(0.01)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        dist_remaining = feedback.distance_remaining
        current_time = time.time()

        if self.initial_distance_remaining is None:
            self.initial_distance_remaining = dist_remaining
            return

        if abs(self.initial_distance_remaining - dist_remaining) > 0.05:
            self.has_moved = True

        elapsed = current_time - self.start_nav_time
        if elapsed >= 3.0 and not self.has_moved:
            if current_time - self.last_warn_time >= 3.0:
                self.get_logger().warn(f'[주의] {elapsed:.1f}초 경과! 로봇이 이동하지 않습니다.')
                self.last_warn_time = current_time

    def send_nav_goal(self, target_name, x, y, yaw=0.0):
        if self.emergency_stop:
            return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        if not self._action_client.wait_for_server(timeout_sec=NAV2_WAIT_SEC):
            self.get_logger().error(
                f'[Nav2 없음] {NAV2_WAIT_SEC:.0f}초 기다려도 navigate_to_pose 서버가 없습니다. '
                'navigation2.launch.py 가 켜져 있는지 확인하세요.'
            )
            return False
        self.get_logger().info(f'>> 이동 시작 요청: {target_name} ({x:.2f}, {y:.2f})')

        self.target_name = target_name
        self.start_nav_time = time.time()
        self.last_warn_time = time.time()
        self.initial_distance_remaining = None
        self.has_moved = False

        send_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        while not send_future.done():
            if self.emergency_stop:
                self.stop_robot_hardware()
                return False
            time.sleep(0.02)

        self.current_goal_handle = send_future.result()

        if not self.current_goal_handle.accepted:
            self.get_logger().error(f'[거절] Nav2가 {target_name} 목표를 거절했습니다.')
            self.current_goal_handle = None
            return False

        result_future = self.current_goal_handle.get_result_async()
        while not result_future.done():
            if self.emergency_stop:
                if self.current_goal_handle is not None:
                    self.current_goal_handle.cancel_goal_async()
                self.stop_robot_hardware()
                return False
            time.sleep(0.02)

        result = result_future.result()
        self.current_goal_handle = None

        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'>> 도달 완료: {target_name}')
            return True
        elif result.status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn(f'>> 주행 취소 완료: {target_name}')
            return False
        else:
            self.get_logger().error(f'>> 주행 실패/중단: 코드 {result.status}')
            return False

    def order_callback(self, msg):
        order_text = msg.data.strip()

        # [1] STOP 명령 수신 시
        if order_text == 'STOP':
            self.get_logger().warn('🚨 [긴급 정지] STOP 토픽 수신! 즉시 하드웨어 제동 및 경로 취소 실행!')
            self.emergency_stop = True
            self.cancel_return_timer()  # [추가] 정지하면 자동 복귀도 안 함

            self.stop_robot_hardware()
            if self.current_goal_handle is not None:
                self.current_goal_handle.cancel_goal_async()
            return

        with self.state_lock:
            if self.is_delivering:
                self.get_logger().warn('현재 작업 중입니다. 신규 주문이 거절되었습니다.')
                return
            self.is_delivering = True
            self.cancel_return_timer()  # [추가] 10초 안에 새 주문 → 자동 복귀 취소, 여기서 바로 출발

        try:
            self.emergency_stop = False
            route = [t.strip() for t in order_text.split(',') if t.strip()]

            # 단일 배달(모드 2) 여부 확인
            is_single_delivery = (len(route) == 1)

            if is_single_delivery:
                display_route = '주방(0번)' if route[0] == '0' else f'{route[0]}번 테이블'
                self.get_logger().info(f"\n[작업 접수] 단일 배달: {display_route} (이동 완료 후 대기)")
            else:
                self.get_logger().info(f"\n[작업 접수] 주행 경로: {' -> '.join(route)} -> 주방")

            success = False  # [추가] 없는 번호만 들어와도 아래에서 에러 안 나게
            for target in route:
                if self.emergency_stop:
                    break
                if target not in WAYPOINTS:
                    self.get_logger().warn(f'[건너뜀] "{target}" 은(는) 등록된 목적지가 아닙니다.')
                    continue

                tx, ty, tyaw = WAYPOINTS[target]

                # 0번일 경우 출력 이름 변경
                display_name = '주방(0번)' if target == '0' else f'{target}번 테이블'

                success = self.send_nav_goal(display_name, tx, ty, tyaw)
                if not success or self.emergency_stop:
                    break

                self.get_logger().info(f'[{display_name}] 도착 완료. 대기 중 ({SERVE_WAIT_SEC:.0f}초)...')

                wait_steps = int(SERVE_WAIT_SEC / 0.05)
                for _ in range(wait_steps):
                    if self.emergency_stop:
                        self.get_logger().warn(f'[{display_name}] 대기 중 비상 정지 감지! 대기 종료.')
                        break
                    time.sleep(0.05)

            # [핵심 수정] 단일 배달(모드 2)은 주방 복귀 생략, 다수 배달만 주방 복귀
            if not self.emergency_stop:
                if is_single_delivery:
                    final_display = '주방(0번)' if route[0] == '0' else f'{route[0]}번 테이블'
                    # [추가] 테이블이면 10초 주문 대기 → 없으면 auto_return_callback 이 주방 복귀
                    if route[0] != '0' and success:
                        self.get_logger().info(
                            f'[{final_display}] 단일 이동 완료. {IDLE_RETURN_SEC:.0f}초 동안 추가 주문 대기 '
                            '(없으면 주방 자동 복귀).\n'
                        )
                        self.return_timer = self.create_timer(
                            IDLE_RETURN_SEC, self.auto_return_callback, callback_group=self.cbg
                        )
                    else:
                        self.get_logger().info(f'[{final_display}] 단일 이동 완료. 해당 위치에서 신규 주문 대기 모드로 전환.\n')
                else:
                    self.get_logger().info('모든 배달 완료. 주방(kitchen)으로 복귀합니다.')
                    kx, ky, kyaw = WAYPOINTS['kitchen']
                    if self.send_nav_goal('주방(kitchen)', kx, ky, kyaw):
                        self.get_logger().info('주방 복귀 완료. 신규 주문 대기 모드로 전환.\n')
                    else:
                        self.get_logger().error('주방 복귀 실패. 로봇 위치를 확인하세요. (신규 주문은 받습니다)\n')
            else:
                self.get_logger().warn('🚨 긴급 정지로 인해 모든 주행 루틴이 중단되었습니다.')
        finally:
            self.target_name = ''
            with self.state_lock:
                self.is_delivering = False


def main(args=None):
    rclpy.init(args=args)
    node = DeliveryManagerNode()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.stop_robot_hardware()
        except Exception:
            pass
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
