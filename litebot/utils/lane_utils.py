#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
    차선 감지 관련 유틸리티 함수들
"""
import numpy as np


def lane_detection(hough, nwindows, width, minpix):
    """
        Sliding Window로 중심선 탐지
        
        Args:
            hough: Hough Line 이미지 (바이너리)
            nwindows: 윈도우 개수
            width: 윈도우 너비 (이미지 너비의 비율 또는 픽셀값)
            minpix: 윈도우 내 최소 픽셀 수
        
        Returns:
            dict: 차선 정보
                - "position": 정규화된 차선 위치 편차 (-1.0 ~ 1.0)
                - "heading": 차선 각도 (라디안)
                - "fit": 2차 다항식 계수
                - "x": x 좌표 리스트
                - "y": y 좌표 리스트
                - "mid_avg": x 좌표 평균
            또는 None (차선을 감지하지 못한 경우)
    """
    h, w = hough.shape
    
    # 하단 절반 영역의 히스토그램으로 중심점 찾기
    histogram = np.sum(hough[h // 2:, :], axis=0)
    midx = np.argmax(histogram)
    
    # 윈도우 너비 계산
    if width < 1.0:
        width = int(w * width)
    margin = width // 2
    
    # 윈도우 높이 계산
    window_height = h // nwindows
    
    # 0이 아닌 픽셀 위치 찾기
    nz = hough.nonzero()
    
    # 중심선 인덱스와 좌표 리스트
    mid_lane_inds = []
    x_list, y_list = [], []
    
    # 윈도우를 위로 올라가며 추적 (하단 4개 윈도우 제외)
    for window in range(nwindows - 4):
        y_low = h - (window + 1) * window_height
        y_high = h - window * window_height
        x_low = midx - margin
        x_high = midx + margin
        
        # 현재 윈도우 내의 유효한 픽셀 찾기
        good_inds = (
            (nz[0] >= y_low)
            & (nz[0] < y_high)
            & (nz[1] >= x_low)
            & (nz[1] < x_high)
        ).nonzero()[0]
        
        mid_lane_inds.append(good_inds)
        
        # 충분한 픽셀이 있으면 중심점 업데이트
        if len(good_inds) > minpix:
            midx = int(np.mean(nz[1][good_inds]))
        
        x_list.append(midx)
        y_list.append((y_low + y_high) / 2.0)
    
    # 최소 3개 점이 있어야 피팅 가능
    if len(x_list) < 3:
        return None
    
    # 2차 다항식 피팅 (y를 독립변수로, x를 종속변수로)
    fit = np.polyfit(y_list, x_list, 2)
    
    # 하단 중심점 계산
    center_x_bottom = np.polyval(fit, h)
    
    # 차선 위치 편차 계산 (이미지 중심으로부터의 거리)
    # 왼쪽이 +, 오른쪽이 - (원본 코드와 반대)
    distance = (w / 2.0) - center_x_bottom
    position = distance / (w / 2.0)  # -1.0 ~ 1.0으로 정규화
    
    # Heading 계산 (기울기 근사)
    heading = np.arctan(fit[1])  # 1차 계수의 arctan
    
    # 예외 처리
    if center_x_bottom == 0:
        position = 0.0
    
    return {
        "heading": heading,
        "position": position,
        "fit": fit,
        "x": x_list,
        "y": y_list,
        "mid_avg": np.mean(x_list),
        "window_width": width,
        "nwindows": nwindows
    }

