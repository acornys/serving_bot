#!/usr/bin/env python3
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
# [수정] 방향 칸 추가. 지금은 전부 0.0 (원래 코드와 같은 동작)
#        → 새 맵에서 /amcl_pose 로 좌표 다시 잴 때 방향도 같이 적기
WAYPOINTS = {
    'kitchen': (0.01, -0.26, 0.0),  # 주방
    '1': (1.59, 0.15, 0.0),         # 1번 테이블
    '2': (3.02, 0.63, 0.0),         # 2번 테이블
    '3': (3.18, -0.82, 0.0),        # 3번 테이블
    '4': (1.05, -1.29, 0.0),        # 4번 테이블
}

NAV2_WAIT_SEC = 5.0  # [수정] Nav2 서버를 기다리는 최대 시간


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
        # [수정] 주문 두 개가 거의 동시에 오면 둘 다 접수되는 것 방지
        self.state_lock = threading.Lock()

        self.start_nav_time = 0.0
        self.last_warn_time = 0.0
        self.initial_distance_remaining = None
        self.has_moved = False
        self.target_name = ''

        self.get_logger().info('서빙 백엔드 매니저 가동 완료 (긴급 정지 및 실시간 좌표 최적화)')

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
        # [수정] 도착 방향(yaw) → 쿼터니언 z, w
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        # [수정] 원래는 wait_for_server() 를 시간 제한 없이 기다려서,
        #        Nav2 가 안 켜져 있으면 영원히 멈추고 STOP 도 안 먹혔음
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

            if self.current_goal_handle is not None:
                self.current_goal_handle.cancel_goal_async()

            self.stop_robot_hardware()
            # is_delivering 강제 초기화 제거됨 (스레드 충돌 방지)
            return

        # [수정] 확인과 표시를 한 번에 (lock) — 동시에 온 주문 두 개가 둘 다 통과하지 않게
        with self.state_lock:
            if self.is_delivering:
                self.get_logger().warn('현재 작업 중입니다. 신규 주문이 거절되었습니다.')
                return
            self.is_delivering = True

        try:
            self.emergency_stop = False
            route = [t.strip() for t in order_text.split(',') if t.strip()]
            self.get_logger().info(f"\n[작업 접수] 주행 경로: {' -> '.join(route)} -> 주방")

            for target in route:
                if self.emergency_stop:
                    break
                if target not in WAYPOINTS:
                    # [수정] 원래는 말없이 건너뜀 → 이유를 로그로
                    self.get_logger().warn(f'[건너뜀] "{target}" 은(는) 등록된 테이블이 아닙니다.')
                    continue

                tx, ty, tyaw = WAYPOINTS[target]
                success = self.send_nav_goal(f'{target}번 테이블', tx, ty, tyaw)
                if not success or self.emergency_stop:
                    break

                self.get_logger().info(f'[{target}번] 도착 완료. 서빙 대기 중 (10초)...')

                wait_steps = 200  # 0.05s * 200 = 10초
                for _ in range(wait_steps):
                    if self.emergency_stop:
                        self.get_logger().warn(f'[{target}번] 대기 중 비상 정지 감지! 대기 종료.')
                        break
                    time.sleep(0.05)

            if not self.emergency_stop:
                self.get_logger().info('모든 배달 완료. 주방(kitchen)으로 복귀합니다.')
                kx, ky, kyaw = WAYPOINTS['kitchen']
                # [수정] 원래는 실패해도 "복귀 완료" 를 찍었음 → 결과를 보고 찍기
                if self.send_nav_goal('주방(kitchen)', kx, ky, kyaw):
                    self.get_logger().info('주방 복귀 완료. 신규 주문 대기 모드로 전환.\n')
                else:
                    self.get_logger().error('주방 복귀 실패. 로봇 위치를 확인하세요. (신규 주문은 받습니다)\n')
            else:
                self.get_logger().warn('🚨 긴급 정지로 인해 모든 주행 루틴이 중단되었습니다.')
        finally:
            # [수정] 중간에 예외가 나도 '작업 중' 에 영원히 갇히지 않게
            self.target_name = ''
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
        # [수정] Ctrl+C 뒤에는 통신이 이미 닫혔을 수 있어서 정지 명령이 에러를 낼 수 있음
        try:
            node.stop_robot_hardware()
        except Exception:
            pass
        # [수정] 순서: 노드 정리 → 통신 종료 (반대로 하면 종료 때 에러가 날 수 있음)
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
