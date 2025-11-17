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
    
    def update_speed_angular(self, linear_x, angular_z):
        """
            선속도와 각속도를 동시에 업데이트
            
            Args:
                linear_x: 선속도 (m/s)
                angular_z: 각속도 (rad/s)
            
            ActionExecutor._execute_update_speed_angular()에서 호출
        """
        raise NotImplementedError("Subclass must implement update_speed_angular() method")
    
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
    
    def rotate_in_place(self, degrees, ang_speed=1.0):
        """
            제자리 회전 (선속도 0, 각속도 유지)
            
            Args:
                degrees (float): 회전 각도 (deg). 양수=좌, 음수=우
                ang_speed (float): 각속도 크기 (rad/s, 양수)
            
            ActionExecutor._execute_rotate()에서 호출
        """
        raise NotImplementedError("Subclass must implement rotate_in_place() method")
    
    def execute_async(self, action):
        """
            액션을 비동기로 실행 (즉시 반환)
            
            Args:
                action: (command, value) 형태의 튜플
                    - command: "drive_forward", "drive_backward", "drive_circle", "rotate" 등
                    - value: 명령에 필요한 값
            
            Returns:
                None (즉시 반환, 백그라운드에서 실행)
        """
        raise NotImplementedError("Subclass must implement execute_async() method")
    
    def is_action_running(self):
        """
            현재 액션이 실행 중인지 확인
            
            Returns:
                bool: 액션이 실행 중이면 True, 아니면 False
        """
        raise NotImplementedError("Subclass must implement is_action_running() method")
    
    def cancel_action(self):
        """
            현재 실행 중인 액션을 취소하고 정지
            
            Returns:
                None
        """
        raise NotImplementedError("Subclass must implement cancel_action() method")