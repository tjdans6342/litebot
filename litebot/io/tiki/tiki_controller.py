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
        pass
    
    def brake(self):
        """
            정지
        """
        # TODO: 실제 Tiki 제어 명령 전송
        print("Tiki: Brake command")
    
    def set_steering(self, value):
        """
            조향 설정
            
            Args:
                value: 조향 값
        """
        # TODO: 실제 Tiki 제어 명령 전송
        print("Tiki: Set steering: {}".format(value))
    
    def drive_forward_distance(self, distance, speed):
        """
            거리 기반 전진
            
            Args:
                distance: 전진할 거리 (m)
                speed: 속도 (m/s)
        """
        # TODO: 실제 Tiki 제어 명령 전송 (거리 기반 제어)
        print("Tiki: Drive forward: {} m at {} m/s".format(distance, speed))
    
    def drive_backward_distance(self, distance, speed):
        """
            거리 기반 후진
            
            Args:
                distance: 후진할 거리 (m)
                speed: 속도 (m/s)
        """
        # TODO: 실제 Tiki 제어 명령 전송 (거리 기반 제어)
        print("Tiki: Drive backward: {} m at {} m/s".format(distance, speed))
    
    def drive_circle_distance(self, distance, speed, angular_velocity):
        """
            거리 기반 원형 주행
            
            Args:
                distance: 주행할 거리 (m)
                speed: 선속도 (m/s)
                angular_velocity: 각속도 (라디안/초)
        """
        # TODO: 실제 Tiki 제어 명령 전송 (거리 기반 제어)
        print("Tiki: Drive circle: {} m at {} m/s with {} rad/s".format(distance, speed, angular_velocity))
