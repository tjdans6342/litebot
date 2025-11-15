#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
ArucoTrigger 클래스
ArUco 마커 감지 시 규칙에 따라 일련의 액션을 반환합니다.
"""
import time


class ArucoTrigger:
    """
        특정 ID의 ArUco 마커가 감지되면 사전에 정의된 액션 시퀀스를 반환합니다.
        각 마커 ID별 쿨다운과 감지 횟수를 추적하여 중복 실행을 방지합니다.
    """
    
    def __init__(self):
        self.rules = {
            0: {
                1: [
                    ("drive_forward", (0.25, 0.2111)), 
                    ("rotate", -90.0), 
                    ("rotate", 90.0),
                    ("capture", {"mode": "object_detection", "suffix": "aruco0_1"}),
                ],
            },
            2: {
                1: [
                    ("drive_forward", (0.3, 0.2111)), ("rotate", -90.0),
                ],
                2: [
                    ("rotate", 90.0), ("drive_forward", (0.5, 0.2111)), ("rotate", 90.0),
                ],
            },
            3: {
                1: [
                    ("drive_forward", (0.4, 0.2111)), ("rotate", 90.0),
                ],
                2: [
                    ("rotate", 0.0),
                ],
                3: [
                    ("drive_forward", (0.45, 0.2111)), ("rotate", -90.0),
                ],
            },
            5: {
                1: [
                    ("drive_forward", (0.35, 0.2111)), ("rotate", -90.0),
                ],
                2: [
                    ("drive_forward", (0.35, 0.2111)), ("rotate", 90.0),
                ],
            },
        }
        # self.default_actions = [("stop", None)]
        self.cooldown_default = 3.0
        self.cooldown_per_id = {
            0: 6.5,
            2: 1.0,
            4: 8.0,
            5: 5.0,
            10: 10.0
        }
        self.last_trigger_times = {}
        self.seen_counts = {}
    
    def step(self, obs):
        """
            관찰 결과를 받아서 액션을 반환합니다.
            
            Args:
                obs: 관찰 결과 딕셔너리 (observer에서 제공)
            
            Returns:
                tuple: (command, value) 형태의 액션
                또는 None
        """
        marker = obs.get("aruco")
        if not marker:
            return None
        
        marker_id = marker.get("id")
        if marker_id is None:
            return None
        
        now = time.time()
        cooldown = self.cooldown_per_id.get(marker_id, self.cooldown_default)
        last = self.last_trigger_times.get(marker_id, 0.0)
        if (now - last) < cooldown:
            return None
        
        nth = self.seen_counts.get(marker_id, 0) + 1
        self.seen_counts[marker_id] = nth
        
        actions = self._get_actions(marker_id, nth)
        if actions:
            self.last_trigger_times[marker_id] = now
            return actions
        return None
    
    def _get_actions(self, marker_id, nth):
        """
            마커 ID와 nth 감지 번호에 해당하는 액션 시퀀스를 반환합니다.
        """
        if marker_id in self.rules:
            rule = self.rules[marker_id]
            if nth in rule:
                return list(rule[nth])
        return None

