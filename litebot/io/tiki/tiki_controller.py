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