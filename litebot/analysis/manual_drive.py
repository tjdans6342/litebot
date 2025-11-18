#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
LiteBot 수동 주행 스크립트 (ROS 버전)

기능:
- 화살표 키로 전/후진 및 좌/우 회전
- Space로 즉시 정지, ESC로 종료
- C 키로 최근 프레임 캡처 (쿨다운 1초, ActionExecutor 활용)
- 실시간 카메라 프레임을 OpenCV 창으로 표시

필수 조건:
- ROS 환경 준비 및 `roscore` 실행
- `pynput` 설치 (`pip install pynput`)
- litebot 패키지를 모듈로 실행할 수 있는 PYTHONPATH 설정
"""
import argparse
import os
import threading
import time
from datetime import datetime

import cv2
import rospy
from pynput import keyboard

from litebot.bot import LiteBot


def _compute_command(state):
    pressed = state["pressed"]
    linear = 0.0
    angular = 0.0

    if keyboard.Key.up in pressed:
        linear += state["linear_speed"]
    if keyboard.Key.down in pressed:
        linear -= state["linear_speed"]
    if keyboard.Key.left in pressed:
        angular += state["angular_speed"]
    if keyboard.Key.right in pressed:
        angular -= state["angular_speed"]

    if keyboard.Key.space in pressed:
        linear = 0.0
        angular = 0.0

    return linear, angular


def _publisher_loop(bot, state):
    while state["running"] and not rospy.is_shutdown():
        with state["lock"]:
            linear, angular = _compute_command(state)
        bot.controller.update_speed_angular(linear, angular)
        time.sleep(state["publish_interval"])
    bot.controller.brake()


def _display_loop(bot, state):
    window_name = "LiteBot Manual Drive"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    try:
        while state["running"] and not rospy.is_shutdown():
            frame = bot.camera.get_frame()
            if frame is not None:
                cv2.imshow(window_name, frame)
            key = cv2.waitKey(1) & 0xFF
            if key == 27:  # ESC
                state["running"] = False
                break
            time.sleep(0.01)
    finally:
        cv2.destroyAllWindows()


def _handle_capture(bot, state):
    now = time.time()
    with state["lock"]:
        if now - state["last_capture_time"] < state["capture_cooldown"]:
            return
        state["last_capture_time"] = now
    frame = bot.camera.get_frame()
    if frame is None:
        print("캡처 실패: 카메라 프레임이 없습니다.")
        return
    

    # for python3
    # os.makedirs(state["capture_dir"], exist_ok=True)

    # for python2
    try:
        os.makedirs(state["capture_dir"])
    except OSError:
        if not os.path.isdir(state["capture_dir"]):
            raise

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
    save_path = os.path.join(state["capture_dir"], "manual_{}.jpg".format(timestamp))
    bot.action_executor.execute(("capture", (frame.copy(), save_path)))
    print("캡처 완료: {}".format(save_path))


def _key_to_char(key):
    try:
        return key.char
    except AttributeError:
        return None


def _on_press(key, bot, state):
    should_capture = False
    with state["lock"]:
        state["pressed"].add(key)
        char = _key_to_char(key)
        if char and char.lower() == "c":
            should_capture = True
    if should_capture:
        _handle_capture(bot, state)


def _on_release(key, state):
    with state["lock"]:
        state["pressed"].discard(key)
    if key == keyboard.Key.esc:
        state["running"] = False
        return False


def parse_args():
    parser = argparse.ArgumentParser(description="LiteBot ROS 수동 주행")
    parser.add_argument("--linear", type=float, default=0.25, help="전/후진 속도 (m/s)")
    parser.add_argument("--angular", type=float, default=1.0, help="좌/우 회전 속도 (rad/s)")
    parser.add_argument("--hz", type=float, default=20.0, help="명령 퍼블리시 주파수 (Hz)")
    parser.add_argument("--mode", choices=["ros"], default="ros", help="LiteBot 모드 (현재 ros만 지원)")
    parser.add_argument("--capture-dir", type=str, default=os.path.join("recordings", "manual_capture"), help="캡처 이미지 저장 경로")
    parser.add_argument("--capture-cooldown", type=float, default=1.0, help="캡처 쿨다운 시간 (초)")
    return parser.parse_args()


def main():
    args = parse_args()

    rospy.init_node("litebot_manual_drive", anonymous=True)
    bot = LiteBot(mode=args.mode)

    state = {
        "linear_speed": args.linear,
        "angular_speed": args.angular,
        "publish_interval": 1.0 / args.hz,
        "pressed": set(),
        "lock": threading.Lock(),
        "running": True,
        "capture_dir": args.capture_dir,
        "capture_cooldown": max(0.1, args.capture_cooldown),
        "last_capture_time": 0.0,
    }

    publisher_thread = threading.Thread(
        target=_publisher_loop,
        args=(bot, state)
        # daemon=True,
    )
    publisher_thread.daemon = True
    publisher_thread.start()

    display_thread = threading.Thread(
        target=_display_loop,
        args=(bot, state)
        # daemon=True,
    )
    display_thread.daemon = True
    display_thread.start()

    print(
        "\nLiteBot 수동 주행 시작\n"
        "↑/↓ : 전진/후진  |  ←/→ : 좌회전/우회전\n"
        "Space : 즉시 정지  |  C : 캡처  |  ESC : 종료\n"
    )

    with keyboard.Listener(
        on_press=lambda key: _on_press(key, bot, state),
        on_release=lambda key: _on_release(key, state),
    ) as listener:
        listener.join()

    state["running"] = False
    publisher_thread.join(timeout=1.0)
    display_thread.join(timeout=1.0)
    bot.controller.brake()
    print("수동 주행 종료")


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

"""
실행 방법 예시:

1. 기본 실행 (기본값 사용):
   python -m litebot.analysis.manual_drive

2. 전진/후진 속도 조절 (기본값: 0.25 m/s):
   python -m litebot.analysis.manual_drive --linear 0.3

3. 회전 속도 조절 (기본값: 1.0 rad/s):
   python -m litebot.analysis.manual_drive --angular 1.5

4. 명령 퍼블리시 주파수 조절 (기본값: 20 Hz):
   python -m litebot.analysis.manual_drive --hz 30

5. 캡처 이미지 저장 경로 지정 (기본값: recordings/manual_capture):
   python -m litebot.analysis.manual_drive --capture-dir recordings/my_captures

6. 캡처 쿨다운 시간 조절 (기본값: 1.0 초):
   python -m litebot.analysis.manual_drive --capture-cooldown 0.5

7. 여러 옵션 조합:
   python -m litebot.analysis.manual_drive --linear 0.3 --angular 1.5 --hz 30

8. 직접 파일 실행 (PYTHONPATH 설정 후):
   python litebot/analysis/manual_drive.py --linear 0.25

주의사항:
- 실행 전에 roscore가 실행 중이어야 합니다.
- pynput 라이브러리가 설치되어 있어야 합니다 (pip install pynput).
- ROS 환경이 제대로 설정되어 있어야 합니다.

조작 방법:
- ↑/↓ : 전진/후진
- ←/→ : 좌회전/우회전
- Space : 즉시 정지
- C : 프레임 캡처 (쿨다운 적용)
- ESC : 종료
"""
