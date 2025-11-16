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
                tuple: (action, source) 형태
                    - action: (command, value)
                    - source: str, 우세한 트리거 이름 (예: "pothole", "lane", "aruco", "qrcode")
                액션이 없는 경우: (None, None)
        """
        if self.current_actions:
            action = self.current_actions.pop(0)
            return action, getattr(self, "_last_source", None)
        
        actions = self._collect_actions(observations)
        if actions:
            self.current_actions = actions
            action = self.current_actions.pop(0)
            return action, getattr(self, "_last_source", None)
        return None, None
    
    def _collect_actions(self, observations):
        """
            트리거 목록을 순서대로 검사하여 첫 번째로 발생한 액션 리스트를 반환합니다.
        """
        for trigger in self.triggers:
            actions = trigger.step(observations)
            action_list = self._to_action_list(actions)
            if action_list:
                # 우세 트리거를 기억해 두고, 액션은 액션만 보관
                self._last_source = self._trigger_name(trigger)
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

    @staticmethod
    def _trigger_name(trigger):
        """
            트리거 인스턴스에서 식별용 이름을 얻습니다.
        """
        name = getattr(trigger, "name", None)
        if isinstance(name, str) and name:
            return name
        cls = trigger.__class__.__name__.lower()
        # 관용적인 매핑
        if "pothole" in cls:
            return "pothole"
        if "lane" in cls:
            return "lane"
        if "aruco" in cls:
            return "aruco"
        if "qrcode" in cls or "qr" in cls:
            return "qrcode"
        return cls

