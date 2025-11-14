#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
LaneTrigger 클래스
차선 정보를 기반으로 주행 액션을 결정합니다.
"""


class LaneTrigger:
    """
    차선 정보를 기반으로 조향 조정 또는 전진 액션을 반환하는 Trigger
    """
    
    def step(self, obs):
        """
        관찰 결과를 받아서 액션을 반환
        
        Args:
            obs: 관찰 결과 딕셔너리
        
        Returns:
            tuple: ("adjust_steering", offset) 또는 ("drive_forward", speed)
            또는 None (액션이 없는 경우)
        """
        lane = obs.get("lane")
        if not lane:
            return None
        
        if abs(lane["offset"]) > 0.1:
            return ("adjust_steering", lane["offset"])
        
        return ("drive_forward", 0.3)

