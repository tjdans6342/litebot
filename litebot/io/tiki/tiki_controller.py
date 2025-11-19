#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
    TikiController 클래스
    Tiki 환경에서 로봇을 제어하는 클래스
"""
import threading
import time
from litebot.io.controller_interface import ControllerInterface

try:
    from tiki.mini import TikiMini
    TIKI_AVAILABLE = True
except ImportError:
    TIKI_AVAILABLE = False
    print("[TikiController] Warning: tiki.mini not available. Using stub implementation.")


class TikiController(ControllerInterface):
    """
        Tiki 환경에서 로봇을 제어하는 클래스
    """
    
    def __init__(self, motor_mode='PID'):
        """
            TikiController 초기화
            
            Args:
                motor_mode: 'PWM' 또는 'PID' (기본값: 'PID')
        """
        if not TIKI_AVAILABLE:
            print("[TikiController] Running in stub mode (tiki.mini not available)")
            self.tiki = None
        else:
            self.tiki = TikiMini()
            # 모터 모드 설정
            if motor_mode == 'PWM':
                self.tiki.set_motor_mode(self.tiki.MOTOR_MODE_PWM)
            else:
                self.tiki.set_motor_mode(self.tiki.MOTOR_MODE_PID)
        
        self.current_speed = 0.0
        self.motor_mode = motor_mode
        
        # 비동기 액션 실행을 위한 상태 변수
        self._action_thread = None
        self._action_running = False
        self._action_lock = threading.Lock()
        
        # 인코더 기반 거리 제어를 위한 상수
        # TODO: 실제 로봇에 맞게 캘리브레이션 필요
        self.encoder_ticks_per_meter = 5555  # 1미터당 인코더 틱 수
    
    def _read_encoder(self, motor_id):
        """
            TikiMini 인코더 값을 안정적으로 읽어 정수로 반환
        """
        if self.tiki is None:
            return 0
        
        try:
            result = self.tiki.get_encoder(motor_id)
            if isinstance(result, (list, tuple)):
                if len(result) > 0:
                    return int(result[0])
                return 0
            return int(result)
        except Exception:
            return 0
    
    def _convert_speed_to_rpm(self, speed_mps):
        """
            m/s 단위 속도를 Tiki RPM 값(-127~127)으로 변환
            
            Args:
                speed_mps: 속도 (m/s)
            
            Returns:
                int: RPM 값 (-127 ~ 127)
        """
        # TODO: 실제 로봇에 맞게 캘리브레이션 필요
        # 예시: 0.5 m/s = 50 RPM 정도로 가정
        max_speed_mps = 0.365  # 최대 속도 (m/s)
        rpm = int((speed_mps / max_speed_mps) * 127)
        return max(-127, min(127, rpm))
    
    def _convert_angular_to_differential(self, linear_x, angular_z):
        """
            선속도와 각속도를 좌우 모터 속도로 변환
            
            Args:
                linear_x: 선속도 (m/s)
                angular_z: 각속도 (rad/s)
            
            Returns:
                tuple: (left_rpm, right_rpm)
        """
        # 차동 주행 모델
        # v_left = v - (w * wheelbase / 2)
        # v_right = v + (w * wheelbase / 2)
        wheelbase = 0.163  # 바퀴 간 거리 (m), TODO: 실제 값으로 수정
        
        v_left = linear_x - (angular_z * wheelbase / 2.0)
        v_right = linear_x + (angular_z * wheelbase / 2.0)
        
        left_rpm = self._convert_speed_to_rpm(v_left)
        right_rpm = self._convert_speed_to_rpm(v_right)
        
        return left_rpm, right_rpm
    
    def brake(self):
        """
            정지
        """
        if self.tiki is None:
            print("[TikiController] Brake command (stub)")
            return
        
        self.tiki.stop()
        self.current_speed = 0.0
    
    def update_speed_angular(self, linear_x, angular_z):
        """
            선속도/각속도 업데이트
            
            Args:
                linear_x: 선속도 (m/s)
                angular_z: 각속도 (rad/s)
        """
        if self.tiki is None:
            print("[TikiController] update v-omega -> v: {}, w: {} (stub)".format(linear_x, angular_z))
            self.current_speed = linear_x
            return
        
        # 차동 주행으로 변환
        left_rpm, right_rpm = self._convert_angular_to_differential(linear_x, angular_z)
        
        # 개별 모터 제어
        self.tiki.set_motor_power(self.tiki.MOTOR_LEFT, left_rpm)
        self.tiki.set_motor_power(self.tiki.MOTOR_RIGHT, right_rpm)
        
        self.current_speed = linear_x
    
    def drive_forward_distance(self, distance, speed):
        """
            거리 기반 전진
            
            Args:
                distance: 전진할 거리 (m)
                speed: 속도 (m/s)
        """
        if self.tiki is None:
            print("[TikiController] Drive forward: {} m at {} m/s (stub)".format(distance, speed))
            return
        
        # 인코더 기반 거리 제어
        initial_left = self._read_encoder(self.tiki.MOTOR_LEFT)
        target_ticks = int(distance * self.encoder_ticks_per_meter)
        
        rpm = self._convert_speed_to_rpm(speed)
        self.tiki.set_motor_power(self.tiki.MOTOR_LEFT, rpm)
        self.tiki.set_motor_power(self.tiki.MOTOR_RIGHT, rpm)
        
        # 목표 거리까지 대기
        while True:
            current_left = self._read_encoder(self.tiki.MOTOR_LEFT)
            traveled_ticks = abs(current_left - initial_left)
            
            if traveled_ticks >= target_ticks:
                self.brake()
                break
            
            time.sleep(0.01)  # 10ms 간격으로 확인
        
        self.current_speed = 0.0
    
    def drive_backward_distance(self, distance, speed):
        """
            거리 기반 후진
            
            Args:
                distance: 후진할 거리 (m)
                speed: 속도 (m/s, 양수)
        """
        if self.tiki is None:
            print("[TikiController] Drive backward: {} m at {} m/s (stub)".format(distance, speed))
            return
        
        # 인코더 기반 거리 제어 (음수 방향)
        initial_left = self._read_encoder(self.tiki.MOTOR_LEFT)
        target_ticks = int(distance * self.encoder_ticks_per_meter)
        
        rpm = -self._convert_speed_to_rpm(speed)  # 음수로 후진
        self.tiki.set_motor_power(self.tiki.MOTOR_LEFT, rpm)
        self.tiki.set_motor_power(self.tiki.MOTOR_RIGHT, rpm)
        
        # 목표 거리까지 대기
        while True:
            current_left = self._read_encoder(self.tiki.MOTOR_LEFT)
            traveled_ticks = abs(current_left - initial_left)
            
            if traveled_ticks >= target_ticks:
                self.brake()
                break
            
            time.sleep(0.01)
        
        self.current_speed = 0.0
    
    def drive_circle_distance(self, distance, speed, angular_velocity):
        """
            거리 기반 원형 주행
            
            Args:
                distance: 주행할 거리 (m)
                speed: 선속도 (m/s)
                angular_velocity: 각속도 (라디안/초)
        """
        if self.tiki is None:
            print("[TikiController] Drive circle: {} m at {} m/s with {} rad/s (stub)".format(
                distance, speed, angular_velocity))
            return
        
        # 차동 주행으로 변환
        left_rpm, right_rpm = self._convert_angular_to_differential(speed, angular_velocity)
        
        # 인코더 기반 거리 제어
        initial_left = self._read_encoder(self.tiki.MOTOR_LEFT)
        target_ticks = int(distance * self.encoder_ticks_per_meter)
        
        self.tiki.set_motor_power(self.tiki.MOTOR_LEFT, left_rpm)
        self.tiki.set_motor_power(self.tiki.MOTOR_RIGHT, right_rpm)
        
        # 목표 거리까지 대기
        while True:
            current_left = self._read_encoder(self.tiki.MOTOR_LEFT)
            traveled_ticks = abs(current_left - initial_left)
            
            if traveled_ticks >= target_ticks:
                self.brake()
                break
            
            time.sleep(0.01)
        
        self.current_speed = 0.0
    
    def rotate_in_place(self, degrees, ang_speed=1.0):
        """
            제자리 회전 (선속도 0, 각속도 유지) - 시간 기반
            
            Args:
                degrees: 회전할 각도 (deg). 양수: 좌회전, 음수: 우회전
                ang_speed: 각속도 크기 (rad/s, 양수)
        """
        if self.tiki is None:
            radians = abs(degrees) * 3.141592653589793 / 180.0
            duration = radians / ang_speed if ang_speed > 0.0 else 0.0
            print("[TikiController] Rotate in place: {} deg at {} rad/s ({} s) (stub)".format(
                degrees, ang_speed, duration))
            return
        
        # 각속도를 차동 주행으로 변환 (선속도 0)
        left_rpm, right_rpm = self._convert_angular_to_differential(0.0, ang_speed)
        
        # 회전 방향에 따라 조정
        if degrees >= 0:  # 좌회전 (왼쪽 역방향, 오른쪽 정방향)
            self.tiki.set_motor_power(self.tiki.MOTOR_LEFT, -left_rpm)
            self.tiki.set_motor_power(self.tiki.MOTOR_RIGHT, right_rpm)
        else:  # 우회전 (왼쪽 정방향, 오른쪽 역방향)
            self.tiki.set_motor_power(self.tiki.MOTOR_LEFT, left_rpm)
            self.tiki.set_motor_power(self.tiki.MOTOR_RIGHT, -right_rpm)
        
        # 시간 기반 회전
        radians = abs(degrees) * 3.141592653589793 / 180.0
        duration = radians / ang_speed if ang_speed > 0.0 else 0.0
        
        time.sleep(duration)
        self.brake()
    
    # ========== 비동기 액션 실행 메서드 ==========
    
    def _run_action(self, action):
        """
            스레드에서 실행되는 액션 실행 메서드
        """
        try:
            cmd, value = action
            if cmd == "drive_forward":
                distance, speed = value
                self.drive_forward_distance(distance, speed)
            elif cmd == "drive_backward":
                distance, speed = value
                self.drive_backward_distance(distance, speed)
            elif cmd == "drive_circle":
                distance, speed, angular_velocity = value
                self.drive_circle_distance(distance, speed, angular_velocity)
            elif cmd == "rotate":
                if isinstance(value, tuple) and len(value) == 2:
                    degrees, ang_speed = value
                else:
                    degrees = value
                    ang_speed = 1.0
                self.rotate_in_place(degrees, ang_speed)
            else:
                print("[TikiController] Unknown async action: {}".format(cmd))
        except Exception as e:
            print("[TikiController] Error in async action: {}".format(e))
        finally:
            with self._action_lock:
                self._action_running = False
                self._action_thread = None
    
    def execute_async(self, action):
        """
            액션을 비동기로 실행 (즉시 반환)
            
            Args:
                action: (command, value) 형태의 튜플
        """
        with self._action_lock:
            # 기존 액션이 실행 중이면 새 액션 무시 (큐 방식)
            if self._action_running and self._action_thread is not None and self._action_thread.is_alive():
                print("[TikiController] Action already running, ignoring new action: {}".format(action[0]))
                return
            
            # 새 액션을 스레드로 실행
            self._action_running = True
            self._action_thread = threading.Thread(target=self._run_action, args=(action,))
            self._action_thread.daemon = True
            self._action_thread.start()
    
    def is_action_running(self):
        """
            현재 액션이 실행 중인지 확인
            
            Returns:
                bool: 액션이 실행 중이면 True
        """
        with self._action_lock:
            return self._action_running and self._action_thread is not None and self._action_thread.is_alive()
    
    def cancel_action(self):
        """
            현재 실행 중인 액션을 취소하고 정지
        """
        with self._action_lock:
            if self._action_running:
                self.brake()
                self._action_running = False
