#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
    TikiSensor 클래스
    Tiki 환경에서 센서 데이터를 읽는 클래스
    배터리, IMU, 인코더 등
"""
try:
    from tiki.mini import TikiMini
    TIKI_AVAILABLE = True
except ImportError:
    TIKI_AVAILABLE = False
    print("[TikiSensor] Warning: tiki.mini not available. Using stub implementation.")


class TikiSensor:
    """
        Tiki 환경에서 센서 데이터를 읽는 클래스
    """
    
    def __init__(self):
        """
            TikiSensor 초기화
        """
        if not TIKI_AVAILABLE:
            print("[TikiSensor] Running in stub mode (tiki.mini not available)")
            self.tiki = None
        else:
            self.tiki = TikiMini()
    
    def get_battery_voltage(self):
        """
            배터리 전압 읽기
            
            Returns:
                float: 전압 (V)
        """
        if self.tiki is None:
            return 0.0
        
        result = self.tiki.get_battery_voltage()
        if result and len(result) > 0:
            return float(result[0])
        return 0.0
    
    def get_battery_current(self):
        """
            배터리 전류 읽기
            
            Returns:
                float: 전류 (A 또는 mA, 문서 확인 필요)
        """
        if self.tiki is None:
            return 0.0
        
        result = self.tiki.get_current()
        if result and len(result) > 0:
            return float(result[0])
        return 0.0
    
    def get_imu(self):
        """
            IMU 센서 값 읽기 (3축 가속도)
            
            Returns:
                tuple: (ax, ay, az) 가속도 값
        """
        if self.tiki is None:
            return (0.0, 0.0, 0.0)
        
        result = self.tiki.get_imu()
        if result and len(result) >= 3:
            return (float(result[0]), float(result[1]), float(result[2]))
        return (0.0, 0.0, 0.0)
    
    def get_encoder(self, motor):
        """
            인코더 값 읽기
            
            Args:
                motor: 'LEFT' 또는 'RIGHT' (또는 tiki.MOTOR_LEFT, tiki.MOTOR_RIGHT)
            
            Returns:
                int: 인코더 카운트
        """
        if self.tiki is None:
            return 0
        
        # 문자열을 상수로 변환
        if isinstance(motor, str):
            if motor.upper() == 'LEFT':
                motor = self.tiki.MOTOR_LEFT
            elif motor.upper() == 'RIGHT':
                motor = self.tiki.MOTOR_RIGHT
        
        result = self.tiki.get_encoder(motor)
        if result and len(result) > 0:
            return int(result[0])
        return 0
    
    def get_encoders(self):
        """
            양쪽 인코더 값 모두 읽기
            
            Returns:
                tuple: (left_encoder, right_encoder)
        """
        left = self.get_encoder('LEFT')
        right = self.get_encoder('RIGHT')
        return (left, right)

