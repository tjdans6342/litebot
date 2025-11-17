#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
    TikiController 클래스
    Tiki 환경에서 로봇을 제어하는 클래스
"""
from litebot.io.controller_interface import ControllerInterface


class TikiController(ControllerInterface):
    """
        Tiki 환경에서 로봇을 제어하는 클래스
    """
    
    def __init__(self):
        """
            TikiController 초기화
        """
        # TODO: Tiki 제어 초기화
        self.current_speed = 0.0
    
    def brake(self):
        """
            정지
        """
        # TODO: 실제 Tiki 제어 명령 전송
        print("[TikiController] Brake command")
    
    def drive_forward_distance(self, distance, speed):
        """
            거리 기반 전진
            
            Args:
                distance: 전진할 거리 (m)
                speed: 속도 (m/s)
        """
        # TODO: 실제 Tiki 제어 명령 전송 (거리 기반 제어)
        print("[TikiController] Drive forward: {} m at {} m/s".format(distance, speed))
    
    def update_speed_angular(self, linear_x, angular_z):
        """
            선속도/각속도 업데이트 (단발 퍼블리시 형태의 스텁)
        """
        # TODO: 실제 Tiki 제어 명령으로 대체
        self.current_speed = linear_x
        print("[TikiController] update v-omega -> v: {}, w: {}".format(linear_x, angular_z))
    
    def drive_backward_distance(self, distance, speed):
        """
            거리 기반 후진
            
            Args:
                distance: 후진할 거리 (m)
                speed: 속도 (m/s)
        """
        # TODO: 실제 Tiki 제어 명령 전송 (거리 기반 제어)
        print("[TikiController] Drive backward: {} m at {} m/s".format(distance, speed))
    
    def drive_circle_distance(self, distance, speed, angular_velocity):
        """
            거리 기반 원형 주행
            
            Args:
                distance: 주행할 거리 (m)
                speed: 선속도 (m/s)
                angular_velocity: 각속도 (라디안/초)
        """
        # TODO: 실제 Tiki 제어 명령 전송 (거리 기반 제어)
        print("[TikiController] Drive circle: {} m at {} m/s with {} rad/s".format(distance, speed, angular_velocity))
    
    def rotate_in_place(self, degrees, ang_speed=1.0):
        """
            제자리 회전 (선속도 0, 각속도 유지) - 시간 기반
        """
        # TODO: 실제 하드웨어 명령으로 대체
        radians = abs(degrees) * 3.141592653589793 / 180.0
        duration = radians / ang_speed if ang_speed > 0.0 else 0.0
        angular = ang_speed if degrees >= 0.0 else -ang_speed
        print("[TikiController] Rotate in place: {} deg at {} rad/s ({} s)".format(degrees, ang_speed, duration))
    
    # ========== 비동기 액션 실행 메서드 ==========
    
    def execute_async(self, action):
        """
            액션을 비동기로 실행 (즉시 반환)
            
            Args:
                action: (command, value) 형태의 튜플
        """
        cmd, value = action
        # Tiki는 이미 비동기이므로, 동기 메서드를 호출하면 자동으로 비동기 실행됨
        # TODO: 실제 Tiki 비동기 API로 대체
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
            # value는 (degrees, ang_speed) 형태
            if isinstance(value, tuple) and len(value) == 2:
                degrees, ang_speed = value
            else:
                degrees = value
                ang_speed = 1.0
            self.rotate_in_place(degrees, ang_speed)
        else:
            print("[TikiController] Unknown async action: {}".format(cmd))
    
    def is_action_running(self):
        """
            현재 액션이 실행 중인지 확인
            
            Returns:
                bool: Tiki는 비동기이므로, 실제 구현 시 Tiki 상태 API로 확인
        """
        # TODO: Tiki 상태 API로 실제 실행 상태 확인
        return False
    
    def cancel_action(self):
        """
            현재 실행 중인 액션을 취소하고 정지
        """
        # TODO: Tiki 취소 API 호출
        self.brake()