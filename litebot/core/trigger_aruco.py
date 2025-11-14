#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
ArucoTrigger 클래스
ArUco 마커를 감지했을 때 정지 액션을 반환합니다.
"""
import time


class ArucoTrigger:
    """
    ArUco 마커를 감지했을 때 정지 액션을 반환하는 Trigger
    쿨다운 시간을 두어 연속 감지를 방지합니다.
    """
    
    def __init__(self):
        """ArucoTrigger 초기화"""
        self.last_detect_time = 0.0
        self.cooldown = 3.0
    
    def step(self, obs):
        """
        관찰 결과를 받아서 액션을 반환
        
        Args:
            obs: 관찰 결과 딕셔너리
        
        Returns:
            tuple: ("stop", None)
            또는 None (액션이 없는 경우)
        """
        marker = obs.get("aruco")
        if marker and (time.time() - self.last_detect_time > self.cooldown):
            self.last_detect_time = time.time()
            return ("stop", None)
        return None

