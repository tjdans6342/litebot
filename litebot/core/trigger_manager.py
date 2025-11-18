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
        각 TriggerManager 인스턴스는 하나의 리소스만 담당합니다.
    """
    
    def __init__(self, resource=None):
        """
            TriggerManager 초기화
            
            Args:
                resource: 리소스 객체 (is_action_running() 메서드를 가진 객체)
                    - motor: Controller 객체 (ROSController 또는 TikiController)
                    - led: LedResource 객체 (나중에 추가 시)
                    - oled: OledResource 객체 (나중에 추가 시)
                    - None: 리소스 독립 액션(capture, qr_command 등)을 담당
        """
        self.triggers = [
            ArucoTrigger(),
            PotholeTrigger(),
            LaneTrigger(),
            QrcodeTrigger()
        ]
        
        # 자신의 리소스에 해당하는 액션을 큐에 저장
        self.current_actions = []
        self.resource = resource
    
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
            
            Note:
                트리거에서 수집한 모든 액션을 큐에 추가합니다.
                현재 리소스가 실행 중이고 큐에 액션이 있으면 기다렸다가 (return None, None)
                끝나면 다음 액션을 순차적으로 내보냅니다.
        """
        # 1. 현재 리소스가 실행 중인지 확인
        # resource 객체가 있고 is_action_running() 메서드가 있으면 호출
        if self.resource is not None and hasattr(self.resource, "is_action_running"):
            if self.resource.is_action_running():
                # 실행 중이면 기다림 (큐에 액션이 있든 없든 대기, 새 액션도 추가하지 않음)
                return None, None
        
        # 2. 큐에 액션이 남아있으면 새 액션을 수집하지 않음 (시퀀스가 끝날 때까지 대기)
        # action들 사이에 self.is_action_running()이 False가 되는 경우가 있을 수 있음
        # 큐가 비어있을 때만 새 액션을 수집하여 큐에 할당
        if not self.current_actions:
            actions = self._collect_actions(observations)
            if actions:
                self.current_actions = actions
        
        # 3. 큐에서 다음 액션을 가져오기
        if self.current_actions:
            action = self.current_actions.pop(0)
            return action, self._last_trigger_name
        
        # 큐가 비어있음
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
                self._last_trigger_name = self._get_trigger_name(trigger)
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
    def _get_trigger_name(trigger):
        """
            트리거 인스턴스에서 식별용 이름을 얻습니다.
        """
        name = getattr(trigger, "name", None)
        if isinstance(name, str) and name:
            return name
        cls = trigger.__class__.__name__.lower()
        # 좀 더 깔끔하게 매핑
        mapping = {
            "pothole": "pothole",
            "lane": "lane",
            "aruco": "aruco",
            "qrcode": "qrcode",
            "qr": "qrcode"
        }
        for key, value in mapping.items():
            if key in cls:
                return value
        return cls

