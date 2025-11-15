#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
    ActionExecutor 클래스
    Trigger에서 반환된 액션을 실제 제어 명령으로 변환하여 실행합니다.
"""
import math

from ai.detection_bridge import DetectionBridge
from litebot.io.ros.ros_controller import ROSController
from litebot.io.tiki.tiki_controller import TikiController
from litebot.core.control.pid_controller import PIDController


class ActionExecutor:
    """
        액션을 받아서 실제 로봇 제어 명령을 실행하는 클래스
    """
    
    def __init__(self, controller):
        """
            ActionExecutor 초기화
            
            Args:
                controller: Controller 객체 (ROSController 또는 TikiController)
        """
        self.controller = controller
        
        # ROS 모드일 때만 PID Controller 초기화
        if isinstance(self.controller, ROSController):
            # PID 파라미터: kp, ki, kd (필요에 따라 조정)
            self.pid_controller = PIDController(kp=0.5, ki=0.0, kd=0.1)
        else:
            self.pid_controller = None
        
        # 제자리 회전 시 사용할 기본 매개변수
        self.turn_speed = 0.15  # m/s
        # 제자리 회전 시 wheel_base(축간거리)로 쓰임 - 필요 없으면 삭제 가능
        self.turn_diameter = 0.3  # m
        self._detection_bridge = None
    
    def execute(self, action):
        """
            액션을 실행합니다.
            
            Args:
                action: (command, value) 형태의 튜플
                    - command: 액션 명령 ("stop", "drive_forward", "drive_backward", "drive_circle", "adjust_steering", "avoid_pothole", "qr_command", "capture")
                    - value: 액션에 필요한 값
        """
        if not action:
            return
        
        cmd, value = action
        
        # cmd에 따라 해당 메서드 호출
        method_name = "_execute_{}".format(cmd)
        if hasattr(self, method_name):
            method = getattr(self, method_name)
            method(value)
        else:
            print("Unknown action command: {}".format(cmd))
    
    # ========== 주행 관련 액션 메서드 ==========
    
    def _execute_update_speed_angular(self, value):
        """
            LaneTrigger 등에서 전달된 선속도/각속도 명령을 실행
            
            Args:
                value: dict 형태로 {"speed": float, "angular": float}
        """
        linear_speed = value["speed"]
        angular = value["angular"]
        
        if isinstance(self.controller, ROSController):
            self.controller.update_speed_angular(linear_speed, angular)
        elif isinstance(self.controller, TikiController):
            pass  # TODO: Tiki 구현 예정

    def _execute_stop(self, value):
        """
            정지 액션 실행
        """
        if isinstance(self.controller, ROSController):
            # ROS 특화 정지 로직
            self.controller.brake()
        elif isinstance(self.controller, TikiController):
            # Tiki 특화 정지 로직
            pass  # TODO: Tiki 구현 예정
    
    def _execute_drive_forward(self, value):
        """
            전진 액션 실행
            
            Args:
                value: (distance, speed) 형태의 튜플
                    - distance: 전진할 거리 (m 단위)
                    - speed: 전진 속도 (m/s 단위)
        """
        if not isinstance(value, tuple) or len(value) != 2:
            print("Invalid value format for drive_forward. Expected (distance, speed)")
            return
        
        distance, speed = value
        
        if speed <= 0:
            print("Speed must be positive")
            return
        
        if isinstance(self.controller, ROSController):
            # ROS 특화 전진 로직 - Controller에 거리 기반 제어 위임
            self.controller.drive_forward_distance(distance, speed)
        elif isinstance(self.controller, TikiController):
            # Tiki 특화 전진 로직
            pass  # TODO: Tiki 구현 예정

    def _execute_drive_backward(self, value):
        """
            후진 액션 실행
            
            Args:
                value: (distance, speed) 형태의 튜플
                    - distance: 후진할 거리 (m 단위)
                    - speed: 후진 속도 (m/s 단위, 양수 값)
        """
        if not isinstance(value, tuple) or len(value) != 2:
            print("Invalid value format for drive_backward. Expected (distance, speed)")
            return
        
        distance, speed = value
        
        if speed <= 0:
            print("Speed must be positive")
            return
        
        if isinstance(self.controller, ROSController):
            # ROS 특화 후진 로직 - Controller에 거리 기반 제어 위임
            self.controller.drive_backward_distance(distance, speed)
        elif isinstance(self.controller, TikiController):
            # Tiki 특화 후진 로직
            pass  # TODO: Tiki 구현 예정

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
            print("Invalid value format for drive_circle. Expected (distance, speed, diameter, arrow)")
            return
        
        distance, speed, diameter, arrow = value
        
        if speed <= 0:
            print("Speed must be positive")
            return
        
        if diameter <= 0:
            print("Diameter must be positive")
            return
        
        if arrow not in ['left', 'right']:
            print("Arrow must be 'left' or 'right'")
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
        
        if isinstance(self.controller, ROSController):
            # ROS 특화 원형 주행 로직 - Controller에 거리 기반 제어 위임
            self.controller.drive_circle_distance(distance, speed, angular_velocity)
        elif isinstance(self.controller, TikiController):
            # Tiki 특화 원형 주행 로직
            pass  # TODO: Tiki 구현 예정
    
    def _execute_rotate(self, value):
        """
            제자리 회전을 수행합니다.
            
            Args:
                value: 회전 각도 (deg). 양수: 좌회전, 음수: 우회전
        """
        if not isinstance(value, (int, float)):
            print("Invalid value for rotate. Expected degrees as number.")
            return
        
        if abs(value) < 1e-3:
            self._execute_stop(None)
            return
        
        radius = self.turn_diameter / 2.0
        speed = self.turn_speed
        distance = (abs(value) / 360.0) * (math.pi * self.turn_diameter)
        angular_velocity = speed / radius
        if value < 0:
            angular_velocity = -angular_velocity
        
        if isinstance(self.controller, ROSController):
            self.controller.drive_circle_distance(distance, speed, angular_velocity)
        elif isinstance(self.controller, TikiController):
            pass  # TODO: Tiki 구현 예정

    def _execute_adjust_steering(self, value):
        """
            조향 조정 액션 실행
            
            Args:
                value: 조향 입력 값 (차선 position 등)
        """
        if isinstance(self.controller, ROSController):
            if self.pid_controller is not None:
                steering_value = self.pid_controller.update(value)
            else:
                steering_value = value
            self.controller.set_steering(steering_value)
        elif isinstance(self.controller, TikiController):
            # Tiki 특화 조향 조정 로직
            pass  # TODO: Tiki 구현 예정
    

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
        print("QR Command received: {}".format(qr_code))
    
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
            print("Invalid value format for capture. Expected tuple or dict.")
            return
        
        if image is None:
            print("Image is None in capture action.")
            return
        
        # --- object_detection 모드: 객체 감지 파이프라인에 이미지 전달 ---
        # mode가 "object_detection"이면 DetectionBridge를 사용해서 세션 이미지 폴더에 저장(큐잉)합니다.
        # 이 이미지는 object_detector.py에 의해 추후 감지 처리됨.
        # {
        #     "image": frame,              # numpy ndarray
        #     "mode": "object_detection",  # 이 값이 있어야 브리지로 전달
        #     "suffix": "pothole",         # 선택: 파일명 끝에 붙는 태그
        #     "base_dir": "detecting"      # 선택: 세션 디렉터리 위치
        # }
        if mode == "object_detection":
            try:
                bridge = self._get_detection_bridge(base_dir)
                saved = bridge.save_frame(image, suffix=suffix)
                print("Image enqueued for object detection: {}".format(saved))
            except Exception as exc:
                print("Failed to enqueue image for detection: {}".format(exc))
            return
        
        # --- 일반 파일 저장 모드: 이미지를 지정 경로에 저장 ---
        # save_path가 반드시 필요하며, 없으면 에러 메시지 반환
        if save_path is None:
            print("save_path is required for file capture mode.")
            return
        
        # 지정된 경로에 폴더가 없다면 생성
        dir_path = os.path.dirname(save_path)
        if dir_path and not os.path.exists(dir_path):
            os.makedirs(dir_path)
        
        # OpenCV를 이용하여 이미지를 파일로 저장. 경로 및 성공 메시지 출력.
        cv2.imwrite(save_path, image)
        print("Image saved to: {}".format(save_path))

    def _get_detection_bridge(self, base_dir=None):
        if base_dir:
            return DetectionBridge(base_dir=base_dir)
        if self._detection_bridge is None:
            self._detection_bridge = DetectionBridge()
        return self._detection_bridge
    
