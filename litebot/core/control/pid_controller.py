#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
PID Controller 클래스
차선 추종을 위한 PID 제어기를 제공합니다.
"""


class PIDController:
    """
    PID (Proportional-Integral-Derivative) 제어기
    오차를 입력받아 제어 값을 계산합니다.
    """
    
    def __init__(self, kp=0.5, ki=0.0, kd=0.1):
        """
        PID Controller 초기화
        
        Args:
            kp: 비례 상수 (Proportional gain)
            ki: 적분 상수 (Integral gain)
            kd: 미분 상수 (Derivative gain)
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        
        # 적분 항과 이전 오차 초기화
        self.integral = 0.0
        self.prev_error = 0.0
    
    def update(self, error):
        """
        PID 제어 값을 계산합니다.
        
        Args:
            error: 현재 오차 (차선 offset 등)
        
        Returns:
            float: 계산된 제어 값
        """
        # 비례 항
        proportional = self.kp * error
        
        # 적분 항 (누적)
        self.integral += error
        
        # 미분 항 (변화율)
        derivative = error - self.prev_error
        self.prev_error = error
        
        # PID 출력
        output = proportional + (self.ki * self.integral) + (self.kd * derivative)
        
        return output
    
    def reset(self):
        """
        PID Controller 상태를 초기화합니다.
        """
        self.integral = 0.0
        self.prev_error = 0.0

