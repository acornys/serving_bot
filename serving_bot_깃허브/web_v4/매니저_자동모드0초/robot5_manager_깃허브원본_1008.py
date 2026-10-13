#!/usr/bin/env python3
# 9/30 테스트 코드 + 모드 2 + 1007 v3 (타임아웃/재시도) + 근접 도착 인정 + 안전성 최종 보완
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
    'kitchen': (0.25, 0.14, 0.0),   # 주방
    '0': (0.25, 0.14, 0.0),         # 0번 (단일 지정 대기용 주방)
    '1': (1.69, 0.05, 0.0),         # 1번 테이블
    '2': (2.88, 0.34, 0.0),         # 2번 테이블
    '3': (2.90, 1.61, 0.0),         # 3번 테이블
    '4': (1.93, 1.63, 0.0),         # 4번 테이블
    '5': (1.07, 1.71, 0.0),        # 5번 테이블
    '6': (0.09, 1.46, 0.0),        # 6번 테이블
}

NAV2_WAIT_SEC = 5.0     # Nav2 서버를 기다리는 최대 시간
SERVE_WAIT_SEC = 3.0    # 테이블 도착 후 대기 시간 (초)
IDLE_RETURN_SEC = 10.0  # 모드 2 도착 후 추가 주문 대기 시간 (초)

# 포기 조건 (임시 값 — 실물에서 맞출 것)
NAV_TIMEOUT_SEC = 60.0  # 한 번 이동 제한 시간 (초). 넘으면 목표 취소
MAX_RECOVERIES = 2      # 복구 동작(빙글빙글 등) 허용 횟수. 넘으면 목표 취소
# 실패했을 때 같은 목표를 다시 보내는 횟수 (0 이면 재시도 안 함)
RETRY_COUNT = 1


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
        self.return_timer = None  # 모드 2 자동 복귀 타이머

        self.start_nav_time = 0.0
        self.last_warn_time = 0.0
        self.initial_distance_remaining = None
        self.latest_distance_remaining = 999.0  # 실시간 거리 추적용 추가
        self.has_moved = False
        self.target_name = ''
        self.recoveries = 0  # 이번 이동에서 Nav2 가 복구 동작을 한 횟수

        self.get_logger().info('서빙 백엔드 매니저 가동 완료 (v3 로직 + 근접 인정/스레드 통합 적용)')

    def cancel_return_timer(self):
        if self.return_timer is not None:
            self.return_timer.cancel()
            self.destroy_timer(self.return_timer)
            self.return_timer = None

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
            if self.go_with_retry('주방(kitchen)', kx, ky, kyaw):
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
        self.latest_distance_remaining = dist_remaining  # 현재 거리 저장
        current_time = time.time()

        # Nav2 가 알려 주는 복구 동작 횟수를 받아 둠
        if hasattr(feedback, 'number_of_recoveries'):
            self.recoveries = feedback.number_of_recoveries

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
        self.latest_distance_remaining = 999.0  # 초기화
        self.has_moved = False
        self.recoveries = 0  # 새 이동 시작할 때 0 으로

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

            # 포기 조건 — 너무 오래 걸리거나 빙글빙글이 반복되면 판단
            too_long = time.time() - self.start_nav_time > NAV_TIMEOUT_SEC
            too_many = self.recoveries > MAX_RECOVERIES
            if too_long or too_many:
                # 타임아웃이나 복구 초과에 걸렸더라도, 이미 테이블 40cm 앞이라면 성공으로 강제 인정!
                if self.has_moved and self.latest_distance_remaining <= 0.40:
                    self.get_logger().info(
                        f'>> [근접 도달 인정] 시간 초과/회전 초과가 발생했으나 목표 근방({self.latest_distance_remaining:.2f}m)에 '
                        f'도달했습니다. 정상 도착으로 처리합니다: {target_name}'
                    )
                    self.current_goal_handle.cancel_goal_async()
                    self.stop_robot_hardware()
                    self.current_goal_handle = None
                    return True
                else:
                    why = f'{NAV_TIMEOUT_SEC:.0f}초 초과' if too_long else f'복구 동작 {self.recoveries}번'
                    self.get_logger().error(f'[포기] {target_name}: {why} → 목표 취소 · 정지')
                    self.current_goal_handle.cancel_goal_async()
                    self.stop_robot_hardware()
                    self.current_goal_handle = None
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
            # Nav2가 각도를 못 맞춰 스스로 포기(ABORTED)했을 때도, 40cm 이내면 도착으로 인정!
            if self.has_moved and self.latest_distance_remaining <= 0.40:
                self.get_logger().info(
                    f'>> [근접 도달 인정] 정밀 각도 정렬은 포기했으나 목표 근방({self.latest_distance_remaining:.2f}m)에 '
                    f'도달했습니다. 정상 도착 처리합니다: {target_name}'
                )
                return True
            else:
                self.get_logger().error(f'>> 주행 실패/중단: 코드 {result.status}')
                return False

    # 실패하면 그 자리에서 같은 목표를 다시 보냄
    def go_with_retry(self, target_name, x, y, yaw=0.0):
        success = self.send_nav_goal(target_name, x, y, yaw)
        for n in range(RETRY_COUNT):
            if success or self.emergency_stop:
                break
            self.get_logger().warn(f'[재시도 {n + 1}/{RETRY_COUNT}] {target_name} 로 다시 출발합니다.')
            time.sleep(1.0)  # 취소 · 정지가 끝날 시간
            
            # [보완 1] 1초 대기 직후 긴급 정지(STOP) 여부를 다시 한번 꼼꼼하게 검사하여 좀비 주행 방지
            if self.emergency_stop:
                break
                
            success = self.send_nav_goal(target_name, x, y, yaw)
        return success

    def order_callback(self, msg):
        order_text = msg.data.strip()

        # [1] STOP 명령 수신 시
        if order_text == 'STOP':
            self.get_logger().warn('🚨 [긴급 정지] STOP 토픽 수신! 즉시 하드웨어 제동 및 경로 취소 실행!')
            self.emergency_stop = True
            self.cancel_return_timer()  # 정지하면 자동 복귀도 안 함

            self.stop_robot_hardware()
            if self.current_goal_handle is not None:
                self.current_goal_handle.cancel_goal_async()
            return

        with self.state_lock:
            if self.is_delivering:
                self.get_logger().warn('현재 작업 중입니다. 신규 주문이 거절되었습니다.')
                return
            self.is_delivering = True
            self.cancel_return_timer()  # 10초 안에 새 주문 → 자동 복귀 취소, 여기서 바로 출발

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

            success = False  # 없는 번호만 들어와도 아래에서 에러 안 나게
            for target in route:
                if self.emergency_stop:
                    break
                if target not in WAYPOINTS:
                    self.get_logger().warn(f'[건너뜀] "{target}" 은(는) 등록된 목적지가 아닙니다.')
                    continue

                tx, ty, tyaw = WAYPOINTS[target]

                # 0번일 경우 출력 이름 변경
                display_name = '주방(0번)' if target == '0' else f'{target}번 테이블'

                # send_nav_goal → go_with_retry (실패하면 다시 시도)
                success = self.go_with_retry(display_name, tx, ty, tyaw)
                if not success or self.emergency_stop:
                    break

                self.get_logger().info(f'[{display_name}] 도착 완료. 대기 중 ({SERVE_WAIT_SEC:.0f}초)...')

                wait_steps = int(SERVE_WAIT_SEC / 0.05)
                for _ in range(wait_steps):
                    if self.emergency_stop:
                        self.get_logger().warn(f'[{display_name}] 대기 중 비상 정지 감지! 대기 종료.')
                        break
                    time.sleep(0.05)

            # 단일 배달(모드 2)은 주방 복귀 생략, 다수 배달만 주방 복귀
            if not self.emergency_stop:
                if is_single_delivery:
                    final_display = '주방(0번)' if route[0] == '0' else f'{route[0]}번 테이블'
                    # 도착 못 했으면 "완료" 대신 [주문 실패]
                    if not success:
                        self.get_logger().error(
                            f'[주문 실패] {final_display} 에 도착하지 못했습니다. '
                            '로봇은 그 자리에 멈춰 있습니다. (신규 주문은 받습니다)\n'
                        )
                    # 테이블이면 10초 주문 대기 → 없으면 auto_return_callback 이 주방 복귀
                    elif route[0] != '0':
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
                    # [보완 2] 다수 경로 배달 중 완전히 막혀서 배달을 망쳤을 경우 억지로 주방으로 가지 않고 강제 대기!
                    if success:
                        self.get_logger().info('모든 배달 완료. 주방(kitchen)으로 복귀합니다.')
                        kx, ky, kyaw = WAYPOINTS['kitchen']
                        # send_nav_goal → go_with_retry
                        if self.go_with_retry('주방(kitchen)', kx, ky, kyaw):
                            self.get_logger().info('주방 복귀 완료. 신규 주문 대기 모드로 전환.\n')
                        else:
                            self.get_logger().error('주방 복귀 실패. 로봇 위치를 확인하세요. (신규 주문은 받습니다)\n')
                    else:
                        self.get_logger().error('[주문 실패] 중간에 멈췄습니다. 억지로 주방(kitchen)으로 돌아가지 않고 대기합니다.')
            else:
                self.get_logger().warn('🚨 긴급 정지로 인해 모든 주행 루틴이 중단되었습니다.')
        finally:
            self.target_name = ''
            with self.state_lock:
                self.is_delivering = False


def main(args=None):
    rclpy.init(args=args)
    node = DeliveryManagerNode()

    # 멀티스레드 병목 현상을 막기 위해 4 -> 10으로 여유 있게 확장
    executor = MultiThreadedExecutor(num_threads=10)
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