#!/usr/bin/env python3
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class OrderUINode(Node):

    def __init__(self):
        super().__init__('order_ui_node')
        self.order_pub = self.create_publisher(String, '/delivery_order', 10)
        # [추가] 매니저가 보내는 로봇 상태 (/delivery_status) — 바뀔 때만 한 줄 출력
        self.last_status = None
        self.status_sub = self.create_subscription(
            String, '/delivery_status', self.status_callback, 10
        )

    def status_callback(self, msg):
        if msg.data != self.last_status:
            self.last_status = msg.data
            print(f'\n   🤖 [로봇 상태] {msg.data}')

    def manager_connected(self):
        """[수정] 매니저(robot5_manager)가 듣고 있는지 확인"""
        return self.order_pub.get_subscription_count() > 0

    def send_order(self, order_str: str):
        """일반 서빙 주문 토픽 발행"""
        if not self.manager_connected():
            # [수정] 원래는 매니저가 꺼져 있어도 "접수" 라고 출력하고 주문이 사라졌음
            print('\n⚠ [전송 실패] 주행 매니저가 연결되어 있지 않습니다. robot5_manager 를 먼저 켜세요.')
            return
        msg = String()
        msg.data = order_str
        self.order_pub.publish(msg)
        # [수정] 매니저가 "작업 중" 이면 거절할 수 있으니 '접수' 대신 '전송'
        print(f'\n>>> [전송 완료] 목적지 [{order_str}] 주문을 주행 매니저에 보냈습니다. (결과는 매니저 창에서 확인)')

    def send_stop(self):
        """긴급 정지 토픽 발행"""
        if not self.manager_connected():
            print('\n⚠ [주의] 주행 매니저가 연결되어 있지 않습니다. 로봇이 움직이면 teleop 이나 전원으로 정지하세요.')
        msg = String()
        msg.data = 'STOP'
        self.order_pub.publish(msg)
        print('\n🚨 [긴급 정지] 터틀봇 즉시 정지 명령을 전송했습니다!')


def print_main_menu():
    print('\n' + '=' * 45)
    print('       [식당 자율주행 서빙 주문 콘솔]       ')
    print('1. 전체 배달 (1 -> 2 -> 3 -> 4)')
    print('2. 단일 배달 (특정 테이블 지정)')
    print('3. 다수 선택 배달 (쉼표 구분 예: 4,2,1)')
    print('4. 주방 복귀 (정지 뒤 돌아오기)')
    print('s. [긴급 정지] 주행 즉시 중단 및 정지')
    print('q. 프로그램 종료')
    print('=' * 45)


def main(args=None):
    rclpy.init(args=args)
    ui_node = OrderUINode()

    spin_thread = threading.Thread(target=rclpy.spin, args=(ui_node,), daemon=True)
    spin_thread.start()

    VALID_TABLES = {'1', '2', '3', '4'}

    try:
        while rclpy.ok():
            print_main_menu()
            user_choice = input('명령을 입력하세요 (1/2/3/4/s/q): ').strip()

            if user_choice.lower() == 's':
                ui_node.send_stop()
                continue

            elif user_choice == '1':
                ui_node.send_order('1,2,3,4')

            elif user_choice == '2':
                while rclpy.ok():
                    target = input('배달할 테이블 번호 (1~4, 취소: c, 정지: s): ').strip()
                    if target.lower() == 's':
                        ui_node.send_stop()
                        break
                    elif target.lower() == 'c':
                        print('[취소] 메인 메뉴로 돌아갑니다.')
                        break
                    elif target in VALID_TABLES:
                        ui_node.send_order(target)
                        break
                    else:
                        print('[입력 오류] 1, 2, 3, 4 중 하나만 입력해주세요.')

            elif user_choice == '3':
                while rclpy.ok():
                    raw = input('배달 순서 입력 (예: 4,2,1 / 취소: c, 정지: s): ').strip()
                    if raw.lower() == 's':
                        ui_node.send_stop()
                        break
                    elif raw.lower() == 'c':
                        print('[취소] 메인 메뉴로 돌아갑니다.')
                        break

                    targets = [t.strip() for t in raw.split(',') if t.strip()]
                    if not (targets and all(t in VALID_TABLES for t in targets)):
                        print('[입력 오류] 1~4 숫자를 쉼표(,)로 올바르게 구분해 입력하세요.')
                    elif len(set(targets)) != len(targets):
                        # [수정] 4,4,1 처럼 같은 테이블이 두 번이면 그 자리에서 10초를 또 기다림
                        print('[입력 오류] 같은 테이블을 두 번 넣었습니다. 한 번씩만 입력하세요.')
                    else:
                        ui_node.send_order(','.join(targets))
                        break

            elif user_choice == '4':
                # [추가] 모드 4 · 주방 복귀 — 매니저에 'HOME' 전송
                ui_node.send_order('HOME')

            elif user_choice.lower() == 'q':
                print('\n주문 콘솔을 종료합니다.')
                break

            else:
                print('[오류] 1, 2, 3, 4, s 또는 q를 입력하세요.')

    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        # [수정] 순서: 노드 정리 → 통신 종료
        ui_node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
