#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
QrcodeTrigger 클래스
QR 코드를 감지했을 때 명령 액션을 반환합니다.
"""
import time


class QrcodeTrigger:
    """
    QR 코드를 감지했을 때 명령 액션을 반환하는 Trigger
    중복 감지를 방지하기 위해 쿨다운과 트리거된 코드를 추적합니다.
    """
    
    def __init__(self, cooldown=5.0):
        """
        QrcodeTrigger 초기화
        
        Args:
            cooldown: 트리거 간 최소 시간 간격 (초)
        """
        self.cooldown = cooldown
        self.last_trigger_time = 0.0
        self.triggered = set()
    
    def step(self, observations):
        """
        관찰 결과를 받아서 액션을 반환
        
        Args:
            observations: 관찰 결과 딕셔너리
        
        Returns:
            tuple: ("qr_command", code)
            또는 None (액션이 없는 경우)
        """
        qr_codes = observations.get("qr_codes")
        if not qr_codes:
            return None
        
        now = time.time()
        if now - self.last_trigger_time < self.cooldown:
            return None
        
        for code in qr_codes:
            if code not in self.triggered:
                self.triggered.add(code)
                self.last_trigger_time = now
                return ("qr_command", code)
        
        return None

