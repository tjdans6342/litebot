#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Observer 클래스
이미지를 받아서 특정 상황을 감지하는 클래스
"""
import cv2
import numpy as np
from litebot.utils import lane_utils


class Observer:
    """
    다양한 상황을 감지하는 Observer 클래스
    차선, ArUco 마커, 포트홀, QR 코드 등을 감지합니다.
    """
    
    def __init__(self):
        """Observer 초기화"""
        # 차선 감지 파라미터
        self.nwindows = 9  # Sliding Window 개수
        self.window_width = 0.1  # 윈도우 너비 (이미지 너비의 비율)
        self.minpix = 50  # 윈도우 내 최소 픽셀 수
    
    def observe_lines(self, image, **kwargs):
        """
        차선을 감지합니다. (Sliding Window 방식)
        
        Args:
            image: Hough Line이 그려진 이미지 (바이너리 이미지)
            **kwargs: 추가 파라미터
        
        Returns:
            dict: 차선 정보
                - "offset": 차선 중심으로부터의 오프셋 (좌: 음수, 우: 양수, -1.0 ~ 1.0)
                - "heading": 차선 각도 (라디안)
            또는 None (차선을 감지하지 못한 경우)
        """
        if image is None:
            return None
        
        # 이미지가 그레이스케일이 아니면 변환
        if len(image.shape) == 3:
            hough = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            hough = image
        
        # Sliding Window로 중심선 탐지
        result = lane_utils.lane_detection(
            hough, 
            nwindows=self.nwindows, 
            width=self.window_width, 
            minpix=self.minpix
        )
        
        if result is None:
            return None
        
        # LaneTrigger가 필요한 형식으로 반환
        return {
            "offset": result["offset"],
            "heading": result.get("heading", 0.0)
        }
    
    def observe_aruco(self, image, **kwargs):
        """
        ArUco 마커를 감지합니다.
        
        Args:
            image: 원본 이미지
            **kwargs: 추가 파라미터
        
        Returns:
            dict: ArUco 마커 정보
                - "id": 마커 ID
                - "corners": 마커의 코너 좌표
                - "center": 마커의 중심 좌표
            또는 None (마커를 감지하지 못한 경우)
        """
        # TODO: 실제 ArUco 감지 로직 구현
        # 현재는 기본 구조만 제공
        return None
    
    def observe_pothole(self, binary_image, **kwargs):
        """
        포트홀을 감지합니다.
        
        Args:
            binary_image: 바이너리 이미지
            **kwargs: 추가 파라미터
        
        Returns:
            bool: 포트홀이 감지되었는지 여부
            또는 dict: 포트홀 정보 (위치, 크기 등)
        """
        # TODO: 실제 포트홀 감지 로직 구현
        # 현재는 기본 구조만 제공
        return False
    
    def observe_qr_codes(self, image, **kwargs):
        """
        QR 코드를 감지합니다.
        
        Args:
            image: 원본 이미지
            **kwargs: 추가 파라미터
        
        Returns:
            list: 감지된 QR 코드들의 리스트 (각 QR 코드는 문자열 또는 dict)
            또는 None (QR 코드를 감지하지 못한 경우)
        """
        # TODO: 실제 QR 코드 감지 로직 구현
        # 현재는 기본 구조만 제공
        return None

