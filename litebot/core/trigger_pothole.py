#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
PotholeTrigger 클래스
포트홀을 감지했을 때 회피 액션을 반환합니다.
"""
import time
import math


class PotholeTrigger:
    """
        포트홀을 감지하면 쿨다운과 감지 횟수에 따라 회피 액션을 실행합니다.
    """
    
    def __init__(self):
        self.cooldown = 5.0
        self.last_trigger_time = 0.0
        self.seen_count = 0
        self.rules = {
            1: [("drive_backward", (0.04, 0.15)), ("rotate", -90.0), ("drive_circle", (0.5 * 0.38 * math.pi, 0.1, 0.38, "left")), ("rotate", -90.0)],
            2: [("drive_backward", (0.04, 0.15)), ("rotate", -90.0), ("drive_circle", (0.5 * 0.38 * math.pi, 0.1, 0.38, "left")), ("rotate", -90.0)],
        }
        self.default_actions = [("stop", None)]
    
    def step(self, obs):
        """
            관찰 결과를 받아서 액션을 반환합니다.
        """
        detected = obs.get("pothole")
        if not detected:
            return None
        
        now = time.time()
        if (now - self.last_trigger_time) < self.cooldown:
            return None
        
        self.seen_count += 1
        self.last_trigger_time = now
        actions = self.rules.get(self.seen_count, self.default_actions)
        return list(actions)

