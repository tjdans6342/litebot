#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Controller Interface
컨트롤러 클래스들이 구현해야 하는 인터페이스
ActionExecutor에서 실제로 호출하는 메서드들만 정의합니다.
"""


class ControllerInterface:
    """
    컨트롤러 인터페이스
    모든 컨트롤러 클래스는 이 인터페이스를 구현해야 합니다.
    ActionExecutor에서 호출하는 메서드들:
    - brake() - 정지
    - set_steering(value) - 조향 설정
    - drive_forward_distance(distance, speed) - 거리 기반 전진
    - drive_backward_distance(distance, speed) - 거리 기반 후진
    - drive_circle_distance(distance, speed, angular_velocity) - 거리 기반 원형 주행
    """
    
    def brake(self):
        """
        정지
        
        ActionExecutor._execute_stop()에서 호출
        """
        raise NotImplementedError("Subclass must implement brake() method")
    
    def set_steering(self, value):
        """
        조향 설정
        
        Args:
            value: 조향 값 (PID 제어 후 값 또는 직접 offset 값)
        
        ActionExecutor._execute_adjust_steering()에서 호출
        """
        raise NotImplementedError("Subclass must implement set_steering() method")
    
    def drive_forward_distance(self, distance, speed):
        """
        거리 기반 전진
        
        Args:
            distance: 전진할 거리 (m)
            speed: 속도 (m/s)
        
        ActionExecutor._execute_drive_forward()에서 호출
        """
        raise NotImplementedError("Subclass must implement drive_forward_distance() method")
    
    def drive_backward_distance(self, distance, speed):
        """
        거리 기반 후진
        
        Args:
            distance: 후진할 거리 (m)
            speed: 속도 (m/s)
        
        ActionExecutor._execute_drive_backward()에서 호출
        """
        raise NotImplementedError("Subclass must implement drive_backward_distance() method")
    
    def drive_circle_distance(self, distance, speed, angular_velocity):
        """
        거리 기반 원형 주행
        
        Args:
            distance: 주행할 거리 (m)
            speed: 선속도 (m/s)
            angular_velocity: 각속도 (라디안/초)
        
        ActionExecutor._execute_drive_circle()에서 호출
        """
        raise NotImplementedError("Subclass must implement drive_circle_distance() method")
