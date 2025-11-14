#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
TriggerManager 클래스
여러 Trigger들을 관리하고 우선순위에 따라 액션을 결정합니다.
"""
from litebot.core.trigger_aruco import ArucoTrigger
from litebot.core.trigger_pothole import PotholeTrigger
from litebot.core.trigger_lane import LaneTrigger
from litebot.core.trigger_qrcode import QrcodeTrigger


class TriggerManager:
    """
    여러 Trigger들을 관리하고 우선순위에 따라 액션을 반환하는 매니저
    """
    
    def __init__(self):
        """TriggerManager 초기화"""
        self.triggers = [
            ArucoTrigger(),
            PotholeTrigger(),
            LaneTrigger(),
            QrcodeTrigger()
        ]
        self.pending_actions = []
    
    def step(self, observations):
        """
        관찰 결과를 받아서 적절한 액션을 반환
        
        Args:
            observations: 관찰 결과 딕셔너리
        
        Returns:
            tuple: 액션 (command, value)
            또는 None (액션이 없는 경우)
        """
        for trigger in self.triggers:
            action = trigger.step(observations)
            if action:
                return action  # 바로 리턴 (우선순위 높은 Trigger 하나만 실행)
        return None

