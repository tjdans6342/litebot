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
        """
            TriggerManager 초기화
        """
        self.triggers = [
            ArucoTrigger(),
            PotholeTrigger(),
            LaneTrigger(),
            QrcodeTrigger()
        ]
        self.current_actions = []
    
    def step(self, observations):
        """
            관찰 결과를 받아서 적절한 액션을 반환합니다.
            
            Args:
                observations: 관찰 결과 딕셔너리
            
            Returns:
                tuple: 액션 (command, value)
                또는 None (액션이 없는 경우)
        """
        if self.current_actions:
            return self.current_actions.pop(0)
        
        actions = self._collect_actions(observations)
        if actions:
            self.current_actions = actions
            return self.current_actions.pop(0)
        return None
    
    def _collect_actions(self, observations):
        """
            트리거 목록을 순서대로 검사하여 첫 번째로 발생한 액션 리스트를 반환합니다.
        """
        for trigger in self.triggers:
            actions = trigger.step(observations)
            action_list = self._to_action_list(actions)
            if action_list:
                return action_list
        return None

    # @staticmethod를 붙이면 self(인스턴스)나 cls(클래스)를 쓰지 않고,
    # 인스턴스나 클래스로부터 직접 호출할 수 있는 메서드가 됩니다.
    # 즉, 인스턴스를 만들 필요 없이 TriggerManager._to_action_list(...)처럼 쓸 수 있습니다.
    # 안 붙이면 self가 반드시 첫 번째 인자로 필요한 '인스턴스 메서드'가 되어,
    # 반드시 객체 생성 후 tm._to_action_list(...)로만 호출할 수 있습니다.
    @staticmethod
    def _to_action_list(actions):
        """
            트리거에서 반환된 액션을 리스트 형태로 통일합니다.
        """
        if isinstance(actions, list):
            return actions
        if isinstance(actions, tuple):
            return [actions]
        return None # 이상한 형태의 액션은 무시

