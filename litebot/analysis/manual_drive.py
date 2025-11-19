#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
LiteBot 수동 주행 스크립트 (ROS/Tiki 모드 지원)

기능:
- 화살표 키로 전/후진 및 좌/우 회전
- Space로 즉시 정지, ESC로 종료
- C 키로 최근 프레임 캡처 (쿨다운 1초, ActionExecutor 활용)
- V 키로 비디오 녹화 시작/중지
- 실시간 카메라 프레임을 OpenCV 창으로 표시

필수 조건:
- `pynput` 설치 (`pip install pynput`)
- ROS 모드 사용 시: ROS 환경 준비 및 `roscore` 실행
- Tiki 모드 사용 시: Tiki 하드웨어 연결
- litebot 패키지를 모듈로 실행할 수 있는 PYTHONPATH 설정
"""
import argparse
import os
import threading
import time
from datetime import datetime

import cv2
from pynput import keyboard

from litebot.bot import LiteBot
from litebot.analysis.video_recorder import VideoRecorder


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
    while state["running"]:
        with state["lock"]:
            linear, angular = _compute_command(state)
        bot.controller.update_speed_angular(linear, angular)
        time.sleep(state["publish_interval"])
    bot.controller.brake()


def _display_loop(bot, state, recorder=None):
    window_name = "LiteBot Manual Drive"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    try:
        while state["running"]:
            frame = bot.camera.get_frame()
            if frame is not None:
                # 비디오 녹화 중이면 프레임 추가
                if recorder is not None and state.get("recording", False):
                    try:
                        recorder.add_frame(frame)
                    except Exception as e:
                        print("[Warning] Failed to add frame to recorder: {}".format(e))
                
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


def _handle_video_recording(bot, state, recorder):
    """비디오 녹화 시작/중지"""
    with state["lock"]:
        if state.get("recording", False):
            # 녹화 중지
            recorder.stop_recording()
            state["recording"] = False
            print("[Video] 녹화 중지: {}".format(recorder.save_path))
        else:
            # 녹화 시작
            if not recorder.is_recording():
                recorder.start_recording()
            state["recording"] = True
            print("[Video] 녹화 시작: {}".format(recorder.save_path))


def _on_press(key, bot, state, recorder=None):
    should_capture = False
    should_toggle_video = False
    with state["lock"]:
        state["pressed"].add(key)
        char = _key_to_char(key)
        if char and char.lower() == "c":
            should_capture = True
        elif char and char.lower() == "v":
            should_toggle_video = True
    if should_capture:
        _handle_capture(bot, state)
    if should_toggle_video and recorder is not None:
        _handle_video_recording(bot, state, recorder)


def _on_release(key, state):
    with state["lock"]:
        state["pressed"].discard(key)
    if key == keyboard.Key.esc:
        state["running"] = False
        return False


def parse_args():
    parser = argparse.ArgumentParser(description="LiteBot 수동 주행 (ROS/Tiki 모드 지원)")
    parser.add_argument("--linear", type=float, default=0.25, help="전/후진 속도 (m/s)")
    parser.add_argument("--angular", type=float, default=1.0, help="좌/우 회전 속도 (rad/s)")
    parser.add_argument("--hz", type=float, default=20.0, help="명령 퍼블리시 주파수 (Hz)")
    parser.add_argument("--mode", choices=["ros", "tiki"], default="ros", help="LiteBot 모드 (ros 또는 tiki)")
    parser.add_argument("--capture-dir", type=str, default=os.path.join("recordings", "manual_capture"), help="캡처 이미지 저장 경로")
    parser.add_argument("--capture-cooldown", type=float, default=1.0, help="캡처 쿨다운 시간 (초)")
    parser.add_argument("--record", action="store_true", help="시작 시 자동으로 비디오 녹화 시작")
    parser.add_argument("--video-path", type=str, default=None, help="비디오 저장 경로 (지정하지 않으면 자동 생성)")
    parser.add_argument("--video-fps", type=float, default=20.0, help="비디오 녹화 FPS")
    return parser.parse_args()


def main():
    args = parse_args()

    # ROS 모드인 경우에만 rospy 초기화
    if args.mode == "ros":
        try:
            import rospy
            rospy.init_node("litebot_manual_drive", anonymous=True)
        except ImportError:
            print("[Error] ROS 모드를 사용하려면 rospy가 필요합니다.")
            return

    bot = LiteBot(mode=args.mode)

    # 비디오 레코더 초기화
    recorder = None
    if args.video_path or args.record:
        recorder = VideoRecorder(save_path=args.video_path, fps=args.video_fps)
        if args.record:
            recorder.start_recording()
            print("[Video] 자동 녹화 시작: {}".format(recorder.save_path))

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
        "recording": args.record,  # 초기 녹화 상태
    }

    publisher_thread = threading.Thread(
        target=_publisher_loop,
        args=(bot, state)
    )
    publisher_thread.daemon = True
    publisher_thread.start()

    display_thread = threading.Thread(
        target=_display_loop,
        args=(bot, state, recorder)
    )
    display_thread.daemon = True
    display_thread.start()

    print(
        "\nLiteBot 수동 주행 시작 (모드: {})\n"
        "↑/↓ : 전진/후진  |  ←/→ : 좌회전/우회전\n"
        "Space : 즉시 정지  |  C : 캡처  |  V : 비디오 녹화 토글  |  ESC : 종료\n"
        .format(args.mode)
    )

    try:
        with keyboard.Listener(
            on_press=lambda key: _on_press(key, bot, state, recorder),
            on_release=lambda key: _on_release(key, state),
        ) as listener:
            listener.join()
    except KeyboardInterrupt:
        print("\n[Interrupted] 사용자에 의해 중단되었습니다.")

    state["running"] = False
    
    # 녹화 중이면 중지
    if recorder is not None and state.get("recording", False):
        recorder.stop_recording()
        print("[Video] 녹화 종료: {}".format(recorder.save_path))
    
    publisher_thread.join(timeout=1.0)
    display_thread.join(timeout=1.0)
    bot.controller.brake()
    print("수동 주행 종료")


if __name__ == "__main__":
    main()

"""
실행 방법 예시:

1. 기본 실행 (ROS 모드, 기본값 사용):
   python -m litebot.analysis.manual_drive

2. Tiki 모드로 실행:
   python -m litebot.analysis.manual_drive --mode tiki

3. 전진/후진 속도 조절 (기본값: 0.25 m/s):
   python -m litebot.analysis.manual_drive --linear 0.3

4. 회전 속도 조절 (기본값: 1.0 rad/s):
   python -m litebot.analysis.manual_drive --angular 1.5

5. 명령 퍼블리시 주파수 조절 (기본값: 20 Hz):
   python -m litebot.analysis.manual_drive --hz 30

6. 캡처 이미지 저장 경로 지정 (기본값: recordings/manual_capture):
   python -m litebot.analysis.manual_drive --capture-dir recordings/my_captures

7. 캡처 쿨다운 시간 조절 (기본값: 1.0 초):
   python -m litebot.analysis.manual_drive --capture-cooldown 0.5

8. 비디오 녹화 시작 시 자동 시작:
   python -m litebot.analysis.manual_drive --record

9. 비디오 저장 경로 지정:
   python -m litebot.analysis.manual_drive --video-path recordings/my_video.avi

10. 비디오 FPS 조절 (기본값: 20.0):
    python -m litebot.analysis.manual_drive --video-fps 30.0

11. 여러 옵션 조합 (Tiki 모드 + 비디오 녹화):
    python -m litebot.analysis.manual_drive --mode tiki --record --linear 0.3

12. 직접 파일 실행 (PYTHONPATH 설정 후):
    python litebot/analysis/manual_drive.py --mode tiki --linear 0.25

주의사항:
- ROS 모드 사용 시: 실행 전에 roscore가 실행 중이어야 합니다.
- Tiki 모드 사용 시: Tiki 하드웨어가 연결되어 있어야 합니다.
- pynput 라이브러리가 설치되어 있어야 합니다 (pip install pynput).

조작 방법:
- ↑/↓ : 전진/후진
- ←/→ : 좌회전/우회전
- Space : 즉시 정지
- C : 프레임 캡처 (쿨다운 적용)
- V : 비디오 녹화 시작/중지 (토글)
- ESC : 종료
"""




# # Tiki 모드로 실행
# python -m litebot.analysis.manual_drive --mode tiki

# # Tiki 모드 + 비디오 녹화 자동 시작
# python -m litebot.analysis.manual_drive --mode tiki --record

# # 비디오 저장 경로 지정
# python -m litebot.analysis.manual_drive --mode tiki --video-path recordings/my_video.avi

# # 여러 옵션 조합
# python -m litebot.analysis.manual_drive --mode tiki --record --linear 0.3 --video-fps 30.0