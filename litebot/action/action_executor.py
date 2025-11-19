#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
    ActionExecutor 클래스
    Trigger에서 반환된 액션을 실제 Tiki 제어 명령으로 변환하여 실행합니다.
    
    주의: Tiki는 한 번 호출하면 계속 실행되므로,
    각 동작 실행 후 반드시 stop()을 호출해야 합니다.
"""
import math
import numpy as np

from ai.detection_bridge import DetectionBridge
from litebot.io.tiki.tiki_controller import TikiController
from litebot.core.control.pid_controller import PIDController
import threading
import time


# 리소스 타입별 액션 분류
RESOURCE_TYPES = {
    "motor": [
        "drive_forward", "drive_backward", "drive_circle",
        "rotate", "stop", "update_speed_angular"
    ],
    "led": [
        "led_on", "led_off", "led_blink", "set_led"
    ],
    "oled": [
        "log", "log_clear"
    ],
    # 센서 읽기는 비동기 액션이 아니므로 제외 (읽기 전용)
}


class ActionExecutor:
    """
        액션을 받아서 실제 로봇 제어 명령을 실행하는 클래스
    """
    
    def __init__(self, controller):
        """
            ActionExecutor 초기화
            
            Args:
                controller: TikiController 객체
        """
        self.controller = controller
        
        # ----- 제어 관련 구성요소 초기화 -----
        # LaneTrigger가 combined_err만 넘기므로, 각속도 제어용 PID만 운용
        self.angular_pid = PIDController(kp=0.65, ki=0.001, kd=0.01)
        self.angular_limit = 1.5   # 각속도 명령 제한 (rad/s)

        # ----- 회전 관련 기본값 -----
        self.turn_speed = 0.15     # 제자리 회전 기본 속도 (m/s)
        self.turn_diameter = 0.3   # 제자리 회전용 가상 바퀴간 거리 (m)
        self.rotate_ang_speed = 1.0  # 제자리 회전 기본 각속도 (rad/s)

        # ----- 검출/내부 -----
        self._detection_bridge = None
        
        # ----- 리소스별 executor (확장 가능) -----
        # 나중에 LED, OLED executor 추가 시 사용
        self.led_executor = None
        self.oled_executor = None
        
        # ----- Tiki update_speed_angular 제어 -----
        # Tiki는 한 번 호출하면 계속 실행되므로, duration 후 stop() 필요
        self._update_speed_thread = None
        self._update_speed_running = False
        self._update_speed_lock = threading.Lock()
        
        # 마지막 motor 액션 추적
        self._last_motor_action = None  # 마지막으로 실행한 motor 액션
    
    def _get_resource_type(self, cmd):
        """
            액션 명령이 사용하는 리소스 타입 반환
        
            Args:
                cmd: 액션 명령 문자열
            
            Returns:
                str: 리소스 타입 ("motor", "led", "oled" 등) 또는 None
        """
        for resource_type, actions in RESOURCE_TYPES.items():
            if cmd in actions:
                return resource_type
        return None  # 리소스가 필요 없는 액션 (capture, qr_command 등)
    
    def execute(self, action):
        """
            액션을 실행합니다.
            
            Args:
                action: tuple
                    # action[0]: 명령어 문자열 (예: "stop", "drive_forward", "drive_backward", "drive_circle", "rotate", "update_speed_angular", "qr_command", "capture")
                    # action[1]: 해당 명령에 필요한 값 (명령별로 포맷 다름)
                    - value: 액션에 필요한 값
            
            Note:
                capture, qr_command는 즉시 실행됩니다 (리소스와 독립).
                리소스 타입별로 실행 제어:
                - motor: drive_forward, rotate, stop, update_speed_angular 등
                - led: led_on, led_off, set_led 등 (확장 가능)
                - oled: log, log_clear 등 (확장 가능)
                같은 리소스 타입의 액션이 실행 중이면 무시됩니다 (큐 방식).
        """
        if not action:
            return
        
        cmd, value = action
        
        # cmd에 따라 해당 메서드 호출
        method_name = "_execute_{}".format(cmd)

        # capture, qr_command는 리소스와 독립적인 액션 (비동기 액션 체크 없이 항상 실행)
        non_controller_actions = ["capture", "qr_command"]
        
        # 리소스 타입 확인
        resource_type = self._get_resource_type(cmd)
        
        # 리소스 타입별 실행 제어
        if resource_type:
            if resource_type == "motor":
                # 모터 리소스는 controller로 체크
                # update_speed_angular 실행 중이면 다른 액션은 무시
                if self.controller.is_action_running():
                    print("[ActionExecutor] Motor resource busy, ignoring: {}".format(cmd))
                    return
                
                # update_speed_angular 스레드 체크 (이중 체크로 안전성 확보)
                with self._update_speed_lock:
                    if self._update_speed_running:
                        print("[ActionExecutor] update_speed_angular running, ignoring: {}".format(cmd))
                        return
                
                # update_speed_angular는 0.05초 동안 보호되어야 하므로
                # 다른 액션이 들어와도 중단하지 않음 (위에서 이미 막힘)
                # line 129, 135에서 체크하므로 여기까지 도달하지 않음
                
                # motor 리소스 액션에서 update_speed_angular가 아닌 경우 PID 리셋
                if method_name != "_execute_update_speed_angular" and self.angular_pid is not None:
                    self.angular_pid.reset()
                
                # 마지막 motor 액션 기록
                self._last_motor_action = cmd
            elif resource_type == "led":
                # LED 리소스는 별도 executor로 체크 (나중에 확장 시)
                if self.led_executor and hasattr(self.led_executor, "is_busy"):
                    if self.led_executor.is_busy():
                        print("[ActionExecutor] LED resource busy, ignoring: {}".format(cmd))
                        return
            elif resource_type == "oled":
                # OLED 리소스는 별도 executor로 체크 (나중에 확장 시)
                if self.oled_executor and hasattr(self.oled_executor, "is_busy"):
                    if self.oled_executor.is_busy():
                        print("[ActionExecutor] OLED resource busy, ignoring: {}".format(cmd))
                        return
        elif cmd not in non_controller_actions:
            # 리소스 타입이 없고 non_controller_actions도 아니면 경고
            print("[ActionExecutor] Unknown action or resource type: {}".format(cmd))

        if hasattr(self, method_name):
            method = getattr(self, method_name)
            method(value) # 실제 액션 실행 메서드 호출
        else:
            print("[ActionExecutor] Unknown action command: {}".format(cmd))
    
    # ========== 주행 관련 액션 메서드 ==========
    
    def _execute_update_speed_angular(self, value):
        """
            LaneTrigger 등에서 전달된 선속도/각속도 명령을 실행
            
            주의: Tiki는 한 번 호출하면 계속 실행되므로,
            update_speed_angular 호출 후 duration(20Hz 기준 0.05초) 실행하고 stop()을 호출해야 합니다.
            
            Args:
                value: dict
                    - LaneTrigger 형식: {"speed": float, "angular": float}  # angular에는 combined_err가 담겨옴
        """
        if not isinstance(value, dict):
            print("[ActionExecutor] Invalid value format for update_speed_angular. Expected dict.")
            return

        linear_speed = float(value.get("speed", 0.0))

        # LaneTrigger는 angular 필드에 combined_err를 담아 보냅니다.
        combined_err = float(value.get("angular", 0.0))

        # PID 계산 (pid_controller가 dt/derivative/적분을 내부에서 처리)
        control = float(self.angular_pid.update(combined_err))
        angular_cmd = np.clip(control, -self.angular_limit, self.angular_limit)

        # 속도가 0이면 즉시 정지
        if linear_speed == 0.0 and angular_cmd == 0.0:
            self._stop_update_speed_angular()
            self.controller.brake()
            self._last_motor_action = "stop"
            return

        # Tiki는 한 번 호출하면 계속 실행되므로, duration 후 stop() 필요
        # 20Hz 기준 0.05초 실행 후 stop() 호출
        duration = 0.05  # 20Hz
        
        # 모터 제어 실행 (내부에서 _update_speed_running = True로 설정됨)
        self.controller.update_speed_angular(linear_speed, angular_cmd)
        self._last_motor_action = "update_speed_angular"
        
        # ActionExecutor에서도 추적 (중복이지만 안전성 확보)
        # duration 후 stop()을 호출하는 스레드 시작
        with self._update_speed_lock:
            # 기존 스레드가 있으면 종료 대기
            if self._update_speed_thread is not None and self._update_speed_thread.is_alive():
                self._update_speed_running = False
                self._update_speed_thread.join(timeout=0.1)
            
            self._update_speed_running = True
            self._update_speed_thread = threading.Thread(
                target=self._update_speed_with_duration,
                args=(duration,),
                daemon=True
            )
            self._update_speed_thread.start()
    
    def _update_speed_with_duration(self, duration):
        """
            duration만큼 대기 후 stop() 호출
        
            Args:
                duration: 대기 시간 (초)
        """
        time.sleep(duration)
        
        with self._update_speed_lock:
            if self._update_speed_running:
                # duration 후 stop() 호출 (Tiki는 한 번 호출하면 계속 실행되므로)
                # brake() 내부에서 TikiController._update_speed_running도 해제됨
                self.controller.brake()
                self._update_speed_running = False
    
    def _stop_update_speed_angular(self):
        """
            update_speed_angular 제어 중단
        """
        with self._update_speed_lock:
            if self._update_speed_running:
                self.controller.brake()
                self._update_speed_running = False
                if self._update_speed_thread is not None and self._update_speed_thread.is_alive():
                    self._update_speed_thread.join(timeout=0.1)

    def _execute_stop(self, value):
        """
            정지 액션 실행
        """
        self._stop_update_speed_angular()
        self.controller.brake()
        self._last_motor_action = "stop"
    
    def _execute_drive_forward(self, value):
        """
            전진 액션 실행 (비동기)
            
            Args:
                value: (distance, speed) 형태의 튜플
                    - distance: 전진할 거리 (m 단위)
                    - speed: 전진 속도 (m/s 단위)
        """
        if not isinstance(value, tuple) or len(value) != 2:
            print("[ActionExecutor] Invalid value format for drive_forward. Expected (distance, speed)")
            return
        
        distance, speed = value
        
        if speed <= 0:
            print("[ActionExecutor] Speed must be positive")
            return
        
        # 비동기로 실행 (즉시 반환)
        # 주의: drive_forward_distance 내부에서 이미 brake()를 호출하므로 안전
        self.controller.execute_async(("drive_forward", (distance, speed)))
        self._last_motor_action = "drive_forward"

    def _execute_drive_backward(self, value):
        """
            후진 액션 실행 (비동기)
            
            Args:
                value: (distance, speed) 형태의 튜플
                    - distance: 후진할 거리 (m 단위)
                    - speed: 후진 속도 (m/s 단위, 양수 값)
        """
        if not isinstance(value, tuple) or len(value) != 2:
            print("[ActionExecutor] Invalid value format for drive_backward. Expected (distance, speed)")
            return
        
        distance, speed = value
        
        if speed <= 0:
            print("[ActionExecutor] Speed must be positive")
            return
        
        # 비동기로 실행 (즉시 반환)
        # 주의: drive_backward_distance 내부에서 이미 brake()를 호출하므로 안전
        self.controller.execute_async(("drive_backward", (distance, speed)))
        self._last_motor_action = "drive_backward"

    def _execute_drive_circle(self, value):
        """
            원형 주행 액션 실행
            
            Args:
                value: (distance, speed, diameter, arrow) 형태의 튜플
                    - distance: 주행할 거리 (m 단위)
                    - speed: 선속도 (m/s 단위)
                    - diameter: 원의 지름 (m 단위)
                    - arrow: 방향 ('left' 또는 'right')
        """
        if not isinstance(value, tuple) or len(value) != 4:
            print("[ActionExecutor] Invalid value format for drive_circle. Expected (distance, speed, diameter, arrow)")
            return
        
        distance, speed, diameter, arrow = value
        
        if speed <= 0:
            print("[ActionExecutor] Speed must be positive")
            return
        
        if diameter <= 0:
            print("[ActionExecutor] Diameter must be positive")
            return
        
        if arrow not in ['left', 'right']:
            print("[ActionExecutor] Arrow must be 'left' or 'right'")
            return
        
        # 반지름 계산
        radius = diameter / 2.0
        
        # 각속도 계산: ω = v / r (라디안/초)
        # 원주 길이 공식: 원주 = 2πr
        # 한 바퀴 도는 데 걸린 시간 t에 대해,
        # 선속도 v = (2πr) / t
        # 각속도 ω = (2π 라디안) / t
        # 두 식을 비교하면: v = ω × r
        # 따라서: ω = v / r
        # 선속도 v = speed, 반지름 r = radius
        angular_velocity = speed / radius
        
        # arrow에 따라 각속도 부호 결정
        # left: 양수 (반시계방향), right: 음수 (시계방향)
        if arrow == 'right':
            angular_velocity = -angular_velocity
        
        # 비동기로 실행 (즉시 반환)
        # 주의: drive_circle_distance 내부에서 이미 brake()를 호출하므로 안전
        self.controller.execute_async(("drive_circle", (distance, speed, angular_velocity)))
        self._last_motor_action = "drive_circle"
    
    def _execute_rotate(self, value):
        """
            제자리 회전을 수행합니다.
            
            Args:
                value: 회전 각도 (deg). 양수: 좌회전, 음수: 우회전
        """
        if not isinstance(value, (int, float)):
            print("[ActionExecutor] Invalid value for rotate. Expected degrees as number.")
            return
        
        if abs(value) < 1e-3:
            self._execute_stop(None)
            return
        
        # 비동기로 실행 (즉시 반환)
        # 주의: rotate_in_place 내부에서 이미 brake()를 호출하므로 안전
        # rotate 명령은 (degrees, ang_speed) 형태로 전달
        self.controller.execute_async(("rotate", (value, self.rotate_ang_speed)))
        self._last_motor_action = "rotate"

    # ========== 기타 액션 메서드 ==========
    
    def _execute_qr_command(self, value):
        """
            QR 코드 명령 액션 실행
        """
        self._handle_qr_command(value)
    
    def _handle_qr_command(self, qr_code):
        """
            QR 코드 명령을 처리합니다.
            
            Args:
                qr_code: QR 코드 내용
        """
        # TODO: QR 코드 명령에 따른 처리 로직 구현
        print("[ActionExecutor] QR Command received: {}".format(qr_code))
    
    def _execute_capture(self, value):
        """
            캡처 액션 실행 - 옵션에 따라 파일 시스템 저장 또는 객체 감지 파이프라인으로 전달
            
            Args:
                value:
                    - tuple(image, save_path): 기존 방식과 동일하게 경로에 저장
                    - dict:
                        image: 필수, numpy.ndarray 프레임
                        save_path: 직접 저장 시 경로
                        mode: "object_detection" 이면 DetectionBridge 사용
                        suffix: object_detection 모드에서 파일명에 덧붙일 문자열
                        base_dir: DetectionBridge 기본 디렉터리 재정의(선택)
        """
        import cv2
        import os
        
        image = None
        save_path = None
        mode = None
        suffix = None
        base_dir = None
        
        if isinstance(value, tuple) and len(value) == 2:
            image, save_path = value
        elif isinstance(value, dict):
            image = value.get("image")
            save_path = value.get("save_path")
            mode = value.get("mode")
            suffix = value.get("suffix")
            base_dir = value.get("base_dir")
        else:
            print("[ActionExecutor] Invalid value format for capture. Expected tuple or dict.")
            return
        
        if image is None:
            print("[ActionExecutor] Image is None in capture action.")
            return
        
        # --- object_detection 모드: 객체 감지 파이프라인에 이미지 전달 ---
        # mode가 "object_detection"이면 DetectionBridge를 사용해서 세션 이미지 폴더에 저장(큐잉)합니다.
        # 이 이미지는 object_detector.py에 의해 추후 감지 처리됨.
        # {
        #     "image": frame,              # numpy ndarray
        #     "mode": "object_detection",  # 이 값이 있어야 브리지로 전달
        #     "suffix": "pothole",         # 선택: 파일명 끝에 붙는 태그
        #     "base_dir": "detects"      # 선택: 세션 디렉터리 위치
        # }
        if mode == "object_detection":
            try:
                bridge = self._get_detection_bridge(base_dir)
                saved = bridge.save_frame(image, suffix=suffix)
                print("[ActionExecutor] Image enqueued for object detection: {}".format(saved))
            except Exception as exc:
                print("[ActionExecutor] Failed to enqueue image for detection: {}".format(exc))
            return
        
        # --- 일반 파일 저장 모드: 이미지를 지정 경로에 저장 ---
        # save_path가 반드시 필요하며, 없으면 에러 메시지 반환
        if save_path is None:
            print("[ActionExecutor] save_path is required for file capture mode.")
            return
        
        # 지정된 경로에 폴더가 없다면 생성
        dir_path = os.path.dirname(save_path)
        if dir_path and not os.path.exists(dir_path):
            os.makedirs(dir_path)
        
        # OpenCV를 이용하여 이미지를 파일로 저장. 경로 및 성공 메시지 출력.
        cv2.imwrite(save_path, image)
        print("[ActionExecutor] Image saved to: {}".format(save_path))

    def _get_detection_bridge(self, base_dir=None):
        if base_dir:
            return DetectionBridge(base_dir=base_dir)
        if self._detection_bridge is None:
            self._detection_bridge = DetectionBridge()
        return self._detection_bridge
    
