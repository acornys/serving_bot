#!/usr/bin/env python3
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class OrderUINode(Node):

    def __init__(self):
        super().__init__('order_ui_node')
        self.order_pub = self.create_publisher(String, '/delivery_order', 10)

    def send_order(self, order_str: str):
        """일반 서빙 주문 토픽 발행"""
        msg = String()
        msg.data = order_str
        self.order_pub.publish(msg)
        print(f'\n>>> [전송 완료] 목적지 [{order_str}] 주문이 주행 매니저에 접수되었습니다.')

    def send_stop(self):
        """긴급 정지 토픽 발행 (어느 단계에서든 호출 가능)"""
        msg = String()
        msg.data = 'STOP'
        self.order_pub.publish(msg)
        print('\n🚨 [긴급 정지] 터틀봇 즉시 정지 명령을 전송했습니다!')


def print_main_menu():
    print('\n' + '=' * 45)
    print('       [식당 자율주행 서빙 주문 콘솔]       ')
    print('1. 전체 순회 배달 (1 -> 2 -> 3 -> 4 -> 5 -> 6)')
    print('2. 단일 이동 (0~6 지정, 도착 후 10초 주문 없으면 주방 복귀)')
    print('3. 다수 선택 배달 (쉼표 구분 예: 6,4,2,0)')
    print('s. [긴급 정지] 주행 즉시 중단 및 정지')
    print('q. 프로그램 종료')
    print('=' * 45)


def main(args=None):
    rclpy.init(args=args)
    ui_node = OrderUINode()

    # 백그라운드 스레드에서 spin을 가동하여 input 블로킹 중에도 DDS 통신 보장
    spin_thread = threading.Thread(target=rclpy.spin, args=(ui_node,), daemon=True)
    spin_thread.start()

    # [수정] 0번 주방 및 5, 6번 테이블까지 유효성 검사 목록 확장
    VALID_TABLES = {'0', '1', '2', '3', '4', '5', '6'}

    try:
        while rclpy.ok():
            print_main_menu()
            user_choice = input('명령을 입력하세요 (1/2/3/s/q): ').strip()

            # [긴급 정지]
            if user_choice.lower() == 's':
                ui_node.send_stop()
                continue

            # 1. 전체 배달 (1번부터 6번까지 순차 배달 후 복귀)
            elif user_choice == '1':
                ui_node.send_order('1,2,3,4,5,6')

            # 2. 단일 이동 (잘못 입력 시 재입력 루프)
            elif user_choice == '2':
                while rclpy.ok():
                    target = input('이동할 목적지 번호 (0~6, 취소: c, 정지: s): ').strip()
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
                        print('[입력 오류] 0, 1, 2, 3, 4, 5, 6 중 하나만 입력해주세요.')

            # 3. 다수 선택 배달 (잘못 입력 시 재입력 루프)
            elif user_choice == '3':
                while rclpy.ok():
                    raw = input('배달 순서 입력 (예: 6,4,2,0 / 취소: c, 정지: s): ').strip()
                    if raw.lower() == 's':
                        ui_node.send_stop()
                        break
                    elif raw.lower() == 'c':
                        print('[취소] 메인 메뉴로 돌아갑니다.')
                        break

                    targets = [t.strip() for t in raw.split(',') if t.strip()]
                    if targets and all(t in VALID_TABLES for t in targets):
                        ui_node.send_order(','.join(targets))
                        break
                    else:
                        print('[입력 오류] 0~6 숫자를 쉼표(,)로 올바르게 구분해 입력하세요.')

            # 프로그램 종료
            elif user_choice.lower() == 'q':
                print('\n주문 콘솔을 종료합니다.')
                break

            else:
                print('[오류] 1, 2, 3, s 또는 q를 입력하세요.')

    except KeyboardInterrupt:
        pass
    finally:
        # 통신 먼저 안전하게 닫고 노드 메모리 해제
        if rclpy.ok():
            rclpy.shutdown()
        ui_node.destroy_node()


if __name__ == '__main__':
    main()