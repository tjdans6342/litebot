#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
LaneTrigger 클래스
차선 정보를 기반으로 주행 액션을 결정합니다.
"""
from collections import deque


class LaneTrigger:
    """ 차선 정보를 기반으로 주행 모드를 선택하고 속도/각속도 명령을 반환하는 Trigger """
    
    def __init__(self, maxlength=20):
        """ LaneTrigger 초기화 """
        self.heading_history = deque(maxlen=maxlength)
        self.position_history = deque(maxlen=maxlength)
        
        # 직선 / 곡선 모드 옵션 (speed, position_weight, heading_weight)
        self.linear_option = (0.1 * 1.5, 0.2 * 1.5, 0.2 * 1.5)
        self.curved_option = (0.1 * 1.5, 0.5 * 1.5, 0.5 * 1.5)
    
    def step(self, obs):
        """
            관찰 결과를 받아서 액션을 반환
            
            Args:
                obs: 관찰 결과 딕셔너리
            
            Returns:
                list: [("update_speed_angular", {"speed": float, "angular": float})]
                또는 None (액션이 없는 경우)
        """
        lane = obs.get("lane")
        if not lane:
            return None
        if not lane.get("exist_lines", True): # 차선이 없으면 액션 반환 X
            return None
        
        heading_err = float(lane.get("heading", 0.0))
        position_err = float(lane.get("position", 0.0))
        self._update_error_queue(heading_err, position_err)
        
        is_linear = True
        for prev_heading, prev_position in zip(self.heading_history, self.position_history):
            if abs(prev_heading) >= 0.3:
                is_linear = False
                break
        
        if is_linear:
            base_speed, position_weight, heading_weight = self.linear_option
            # print("linear_mode")
        else:
            base_speed, position_weight, heading_weight = self.curved_option
            # print("curve_mode")
        
        # print(position_weight, position_err, heading_weight, heading_err)

        combined_err = (position_weight * position_err) + (heading_weight * heading_err)
        return [("update_speed_angular", {"speed": base_speed, "angular": combined_err})]
    
    def _update_error_queue(self, heading_err, position_err):
        """ 최근 차선 편차를 큐에 저장 """
        self.heading_history.append(heading_err)
        self.position_history.append(position_err)

