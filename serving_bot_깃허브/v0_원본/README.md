# 7팀 식당 서빙봇 (ROS 2 Humble · TurtleBot3 burger)

## 받기 · 빌드
```bash
mkdir -p ~/serving_ws/src && cd ~/serving_ws/src && git clone https://github.com/acornys/serving_bot.git
cd ~/serving_ws && colcon build --symlink-install --packages-select serving_bot && source install/setup.bash
```

## 실행 (브링업 · Nav2 · 2D Pose Estimate 뒤)
```bash
ros2 run serving_bot manager
ros2 run serving_bot order_ui
```
콘솔: `1` 전체 · `2` 단일 · `3` 골라서 · `4` 주방 복귀 · `s` 긴급 정지

## 버전
- v0 원본 (팀원 코드)
- v1 1차 수정 — 멈추거나 거짓 표시하던 곳 고침
- v2 2차 수정 — 대기 3초 · 모드 4 · 배터리 경고 · 상태 알리기

Commits 화면에서 버전 사이 바뀐 줄을 볼 수 있어요. 코드에서 `# [수정]` · `# [추가]` 검색.

## 규칙
- 고치기 전에 `git pull` · 한 파일은 한 명만 · 로봇으로 테스트한 것만 올리기
- 켜기(브링업 · 매니저)는 한 명만