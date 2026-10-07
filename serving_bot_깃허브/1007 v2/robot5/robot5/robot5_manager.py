#!/usr/bin/env python3
# 9/30 테스트 코드 + 모드 2 + 2번<->3번 경유지 근방 도달 시 즉시 조기 통과(Early-Pass)
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

# (x, y, yaw)
WAYPOINTS = {
    'kitchen': (0.23, 0.30, 0.0),   # 주방
    '0': (0.23, 0.30, 0.0),         # 0번 (단일 지정 대기용 주방)
    '1': (1.52, 0.33, 0.0),         # 1번 테이블
    '2': (2.74, 0.82, 0.0),         # 2번 테이블
    'wp_2_3': (1.71, 1.40, 0.0),    # 2번 <-> 3번 연결용 중간 경유지
    '3': (2.64, 2.35, 0.0),         # 3번 테이블
    '4': (1.73, 2.09, 0.0),         # 4번 테이블
    '5': (-0.33, 1.73, 0.0),        # 5번 테이블
    '6': (-0.20, 1.15, 0.0),        # 6번 테이블
}

NAV2_WAIT_SEC = 5.0     # Nav2 서버를 기다리는 최대 시간
SERVE_WAIT_SEC = 3.0    # 테이블 도착 후 대기 시간 (초)
IDLE_RETURN_SEC = 10.0  # 모드 2 도착 후 추가 주문 대기 시간 (초)
WAYPOINT_PASS_DIST = 0.30  # [추가] 경유지 통과 인정 반경 (30cm 근방 도달 시 즉시 통과)


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
        self.return_timer = None

        self.start_nav_time = 0.0
        self.last_warn_time = 0.0
        self.initial_distance_remaining = None
        self.latest_distance_remaining = 999.0  # [추가] 실시간 남은 거리 추적용
        self.has_moved = False
        self.target_name = ''
        self.last_visited = 'kitchen'

        self.get_logger().info('서빙 백엔드 매니저 가동 (경유지 조기 통과 기능 활성화)')

    def cancel_return_timer(self):
        if self.return_timer is not None:
            self.return_timer.cancel()
            self.destroy_timer(self.return_timer)
            self.return_timer = None

    def auto_return_callback(self):
        self.cancel_return_timer()
        with self.state_lock:
            if self.is_delivering:
                return
            self.is_delivering = True
        try:
            self.emergency_stop = False
            self.get_logger().info(f'{IDLE_RETURN_SEC:.0f}초 동안 추가 주문 없음 → 주방으로 자동 복귀합니다.')
            kx, ky, kyaw = WAYPOINTS['kitchen']
            if self.send_nav_goal('주방(kitchen)', kx, ky, kyaw):
                self.last_visited = 'kitchen'
                self.get_logger().info('주방 복귀 완료. 신규 주문 대기 모드로 전환.\n')
            else:
                self.get_logger().error('주방 복귀 실패. 로봇 위치를 확인하세요. (신규 주문은 받습니다)\n')
        finally:
            self.target_name = ''
            with self.state_lock:
                self.is_delivering = False

    def print_current_position_timer(self):
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            curr_x = t.transform.translation.x
            curr_y = t.transform.translation.y
            status_text = f'[{self.target_name} 주행 중]' if self.is_delivering else '[주문 대기 중]'
            self.get_logger().info(f'{status_text} 현재 좌표 -> X: {curr_x:.2f} m, Y: {curr_y:.2f} m')
        except TransformException:
            pass

    def stop_robot_hardware(self):
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        for _ in range(15):
            self.cmd_vel_pub.publish(stop_msg)
            time.sleep(0.01)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        dist_remaining = feedback.distance_remaining
        self.latest_distance_remaining = dist_remaining  # 실시간 거리 갱신
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

    def send_nav_goal(self, target_name, x, y, yaw=0.0, is_waypoint=False):
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
            self.get_logger().error(f'[Nav2 없음] navigate_to_pose 서버를 찾을 수 없습니다.')
            return False

        self.get_logger().info(f'>> 이동 시작 요청: {target_name} ({x:.2f}, {y:.2f})')

        self.target_name = target_name
        self.start_nav_time = time.time()
        self.last_warn_time = time.time()
        self.initial_distance_remaining = None
        self.latest_distance_remaining = 999.0
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

            # [핵심] 경유지인 경우: 남은 거리가 반경(30cm) 안으로 들어오면 즉시 목표 취소 후 바로 성공 리턴
            if is_waypoint and self.has_moved and self.latest_distance_remaining <= WAYPOINT_PASS_DIST:
                self.get_logger().info(
                    f'>> [경유지 도달] 남은 거리 {self.latest_distance_remaining:.2f}m <= {WAYPOINT_PASS_DIST}m! '
                    f'정렬 없이 즉시 다음 목표로 주행합니다.'
                )
                self.current_goal_handle.cancel_goal_async()
                self.current_goal_handle = None
                return True  # 바로 다음 목표 진행

            time.sleep(0.02)

        result = result_future.result()
        self.current_goal_handle = None

        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'>> 도달 완료: {target_name}')
            return True
        elif result.status == GoalStatus.STATUS_CANCELED:
            # 외부 STOP으로 인한 취소
            self.get_logger().warn(f'>> 주행 취소: {target_name}')
            return False
        else:
            self.get_logger().error(f'>> 주행 실패/중단: 코드 {result.status}')
            return False

    def build_expanded_route(self, raw_route):
        expanded = []
        curr = self.last_visited
        for nxt in raw_route:
            if (curr == '2' and nxt == '3') or (curr == '3' and nxt == '2'):
                expanded.append('wp_2_3')
            expanded.append(nxt)
            curr = nxt
        return expanded

    def order_callback(self, msg):
        order_text = msg.data.strip()

        if order_text == 'STOP':
            self.get_logger().warn('🚨 [긴급 정지] STOP 토픽 수신! 즉시 하드웨어 제동 및 경로 취소 실행!')
            self.emergency_stop = True
            self.cancel_return_timer()

            self.stop_robot_hardware()
            if self.current_goal_handle is not None:
                self.current_goal_handle.cancel_goal_async()
            return

        with self.state_lock:
            if self.is_delivering:
                self.get_logger().warn('현재 작업 중입니다. 신규 주문이 거절되었습니다.')
                return
            self.is_delivering = True
            self.cancel_return_timer()

        try:
            self.emergency_stop = False
            raw_route = [t.strip() for t in order_text.split(',') if t.strip()]
            route = self.build_expanded_route(raw_route)

            is_single_delivery = (len(raw_route) == 1)

            if is_single_delivery:
                display_route = '주방(0번)' if raw_route[0] == '0' else f'{raw_route[0]}번 테이블'
                self.get_logger().info(f"\n[작업 접수] 단일 배달: {display_route} (이동 완료 후 대기)")
            else:
                self.get_logger().info(f"\n[작업 접수] 실제 주행 경로: {' -> '.join(route)} -> 주방")

            success = False
            for target in route:
                if self.emergency_stop:
                    break
                if target not in WAYPOINTS:
                    self.get_logger().warn(f'[건너뜀] "{target}" 은(는) 등록된 목적지가 아닙니다.')
                    continue

                tx, ty, tyaw = WAYPOINTS[target]
                is_wp = (target == 'wp_2_3')

                if target == '0':
                    display_name = '주방(0번)'
                elif is_wp:
                    display_name = '중간 경유지(2-3 코너)'
                else:
                    display_name = f'{target}번 테이블'

                # [is_waypoint 전달]
                success = self.send_nav_goal(display_name, tx, ty, tyaw, is_waypoint=is_wp)
                if not success or self.emergency_stop:
                    break

                self.last_visited = target

                # 경유지는 손님 서빙 대기 없이 바로 다음 테이블로 직진
                if is_wp:
                    continue

                self.get_logger().info(f'[{display_name}] 도착 완료. 대기 중 ({SERVE_WAIT_SEC:.0f}초)...')
                wait_steps = int(SERVE_WAIT_SEC / 0.05)
                for _ in range(wait_steps):
                    if self.emergency_stop:
                        self.get_logger().warn(f'[{display_name}] 대기 중 비상 정지 감지! 대기 종료.')
                        break
                    time.sleep(0.05)

            # 모드 2 단일 배달 대기 및 다수 배달 복귀
            if not self.emergency_stop:
                if is_single_delivery:
                    final_target = raw_route[0]
                    final_display = '주방(0번)' if final_target == '0' else f'{final_target}번 테이블'
                    if final_target != '0' and success:
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
                        self.last_visited = 'kitchen'
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
        rclpy.shutdown()


if __name__ == '__main__':
    main()