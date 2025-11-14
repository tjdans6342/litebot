#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
PotholeTrigger 클래스
포트홀을 감지했을 때 회피 액션을 반환합니다.
"""


class PotholeTrigger:
    """
    포트홀을 감지했을 때 회피 액션을 반환하는 Trigger
    """
    
    def step(self, obs):
        """
        관찰 결과를 받아서 액션을 반환
        
        Args:
            obs: 관찰 결과 딕셔너리
        
        Returns:
            tuple: ("avoid_pothole", None)
            또는 None (액션이 없는 경우)
        """
        if obs.get("pothole"):
            return ("avoid_pothole", None)
        return None

