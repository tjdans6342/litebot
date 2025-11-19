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
- ROS 모드 사용 시: ROS 환경 준비 및 `roscore` 실행
- Tiki 모드 사용 시: Tiki 하드웨어 연결
- litebot 패키지를 모듈로 실행할 수 있는 PYTHONPATH 설정
- OpenCV 창이 표시되어야 하므로 DISPLAY 환경 변수 설정 필요 (SSH의 경우 X11 forwarding)
"""
import argparse
import os
import threading
import time
from datetime import datetime

import cv2

from litebot.bot import LiteBot
from litebot.analysis.video_recorder import VideoRecorder


def _compute_command(state):
    """현재 눌린 키에 따라 선속도/각속도 계산"""
    pressed_keys = state["pressed"]
    linear = 0.0
    angular = 0.0

    # 화살표 키 또는 WASD 키 지원
    # OpenCV 키 코드: 82=↑, 84=↓, 81=←, 83=→
    # 또는 일반 키: w/W=전진, s/S=후진, a/A=좌회전, d/D=우회전
    up_keys = [82, ord('w'), ord('W')]  # Up arrow or W
    down_keys = [84, ord('s'), ord('S')]  # Down arrow or S
    left_keys = [81, ord('a'), ord('A')]  # Left arrow or A
    right_keys = [83, ord('d'), ord('D')]  # Right arrow or D
    
    for key in up_keys:
        if key in pressed_keys:
            linear += state["linear_speed"]
            break
    for key in down_keys:
        if key in pressed_keys:
            linear -= state["linear_speed"]
            break
    for key in left_keys:
        if key in pressed_keys:
            angular += state["angular_speed"]
            break
    for key in right_keys:
        if key in pressed_keys:
            angular -= state["angular_speed"]
            break

    if 32 in pressed_keys:  # Space
        linear = 0.0
        angular = 0.0

    return linear, angular


def _publisher_loop(bot, state):
    """모터 제어 루프"""
    while state["running"]:
        with state["lock"]:
            linear, angular = _compute_command(state)
            state["current_linear"] = linear
            state["current_angular"] = angular
        bot.controller.update_speed_angular(linear, angular)
        time.sleep(state["publish_interval"])
    bot.controller.brake()


def _display_loop(bot, state, recorder=None):
    """디스플레이 및 키 입력 처리 루프"""
    window_name = "LiteBot Manual Drive"
    
    # DISPLAY 환경 변수 확인
    display = os.environ.get('DISPLAY')
    if not display:
        print("[Warning] DISPLAY 환경 변수가 설정되지 않았습니다.")
        print("[Warning] OpenCV 창을 표시할 수 없습니다. 다음 중 하나를 시도하세요:")
        print("  1. SSH 접속 시 X11 forwarding 사용: ssh -X jetson@jetson-desktop")
        print("  2. DISPLAY 환경 변수 설정: export DISPLAY=:0")
        print("  3. Jetson에 직접 모니터 연결 후 실행")
        print("\n[Error] GUI를 사용할 수 없어 종료합니다.")
        state["running"] = False
        return
    
    # X 서버 연결 테스트
    try:
        import subprocess
        result = subprocess.run(['xset', 'q'], 
                              capture_output=True, 
                              timeout=2)
        if result.returncode != 0:
            print("[Warning] X 서버 연결 테스트 실패. DISPLAY={}".format(display))
            print("[Info] X 서버 연결을 확인하는 중...")
    except (subprocess.TimeoutExpired, FileNotFoundError, Exception) as e:
        print("[Warning] X 서버 연결 확인 실패: {}".format(e))
    
    try:
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    except cv2.error as e:
        print("[Error] OpenCV 창을 생성할 수 없습니다: {}".format(e))
        print("[Error] DISPLAY={} 설정을 확인하세요.".format(display))
        print("\n[해결 방법]")
        print("1. Windows에서 VcXsrv가 실행 중인지 확인")
        print("2. VcXsrv 설정에서 'Disable access control'이 체크되어 있는지 확인")
        print("3. Windows 방화벽에서 VcXsrv 허용 확인")
        print("4. DISPLAY 재설정 시도:")
        print("   - Windows IP 주소 확인: ipconfig (Windows에서)")
        print("   - export DISPLAY=<Windows_IP>:0.0")
        print("   예: export DISPLAY=192.168.50.100:0.0")
        print("5. 또는 Jetson에 직접 모니터 연결 후 실행")
        state["running"] = False
        return
    
    # 키 입력 상태 추적 (OpenCV는 키가 눌린 상태를 직접 추적하지 않으므로 수동 관리)
    last_key_time = {}  # 키 코드 -> 마지막 눌린 시간
    
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
                
                # 상태 표시 텍스트 추가
                status_text = "Mode: {} | Linear: {:.2f} | Angular: {:.2f}".format(
                    state.get("mode", "unknown"),
                    state.get("current_linear", 0.0),
                    state.get("current_angular", 0.0)
                )
                if state.get("recording", False):
                    status_text += " | [REC]"
                cv2.putText(frame, status_text, (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
                cv2.imshow(window_name, frame)
            
            # 키 입력 처리 (OpenCV waitKey는 키가 눌렸을 때만 반환)
            key = cv2.waitKey(1) & 0xFF
            current_time = time.time()
            
            with state["lock"]:
                # 키가 눌렸는지 확인
                if key != 255:  # 255는 키가 눌리지 않았을 때
                    last_key_time[key] = current_time
                    state["pressed"].add(key)
                    
                    # 특수 키 처리
                    if key == 27:  # ESC
                        state["running"] = False
                        break
                    elif key == ord('c') or key == ord('C'):  # C 키
                        _handle_capture(bot, state)
                    elif key == ord('v') or key == ord('V'):  # V 키
                        if recorder is not None:
                            _handle_video_recording(bot, state, recorder)
                
                # 키가 일정 시간 이상 눌리지 않으면 released로 간주
                # (연속 입력을 위해 짧은 시간 동안은 유지)
                keys_to_remove = []
                for k in state["pressed"]:
                    if k in last_key_time:
                        if current_time - last_key_time[k] > 0.15:  # 150ms 후 release
                            keys_to_remove.append(k)
                    else:
                        keys_to_remove.append(k)
                for k in keys_to_remove:
                    state["pressed"].discard(k)
            
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
        "pressed": set(),  # OpenCV 키 코드 집합
        "lock": threading.Lock(),
        "running": True,
        "capture_dir": args.capture_dir,
        "capture_cooldown": max(0.1, args.capture_cooldown),
        "last_capture_time": 0.0,
        "recording": args.record,  # 초기 녹화 상태
        "mode": args.mode,
        "current_linear": 0.0,
        "current_angular": 0.0,
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
        "조작 방법:\n"
        "  ↑/W : 전진\n"
        "  ↓/S : 후진\n"
        "  ←/A : 좌회전\n"
        "  →/D : 우회전\n"
        "  Space : 즉시 정지\n"
        "  C : 프레임 캡처\n"
        "  V : 비디오 녹화 토글\n"
        "  ESC : 종료\n"
        "\n주의: OpenCV 창이 포커스되어 있어야 키 입력이 인식됩니다.\n"
        "      화살표 키가 작동하지 않으면 W/A/S/D 키를 사용하세요.\n"
        .format(args.mode)
    )

    try:
        # display_thread가 메인 루프 역할을 함
        display_thread.join()
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