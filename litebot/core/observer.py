#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Observer 클래스
이미지를 받아서 특정 상황을 감지하는 클래스
"""
import cv2
import numpy as np
from litebot.utils import lane_utils, check_utils, aruco_utils


class Observer:
    """
        다양한 상황을 감지하는 Observer 클래스
        차선, ArUco 마커, 포트홀, QR 코드 등을 감지합니다.
    """
    
    def __init__(self):
        """
            Observer 초기화
        """
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
                    - "position": 차선 중심으로부터의 편차 (좌: 음수, 우: 양수, -1.0 ~ 1.0)
                    - "heading": 차선 각도 (라디안)
                    - "fit": 2차 곡선 계수
                    - "x", "y": 슬라이딩 윈도우 중심 좌표 리스트
                    - "mid_avg": 윈도우 중심 평균 x 좌표
                    - "window_width": 슬라이딩 윈도우 너비 (px)
                    - "nwindows": 사용한 윈도우 개수
                    - "exist_lines": 차선 존재 여부 (False이면 다른 키가 없을 수 있음)
                또는 None (차선을 감지하지 못한 경우)
        """
        if image is None:
            return None
        
        # 이미지가 그레이스케일이 아니면 오류 발생
        if len(image.shape) == 3:
            raise ValueError("입력 이미지는 반드시 그레이스케일(단일 채널)이어야 합니다.")
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
        
        # sliding window 결과 그대로 사용 (position, heading 외 보조 정보 포함)
        return result
    
    def observe_aruco(self, image, **kwargs):
        """ 아루코 마커 감지 (세부 설명은 aruco_utils.detect_largest_marker 참고) """
        aruco_dict_name = kwargs.get("aruco_dict", "DICT_4X4_50")
        detector_params = kwargs.get("detector_params")
        return aruco_utils.detect_largest_marker( # 가장 큰 아루코 마커만 감지하는 함수
            image,
            aruco_dict=aruco_dict_name,
            detector_params=detector_params
        )
    
    def observe_pothole(self, binary_image, **kwargs):
        """
            포트홀을 감지합니다.
            
            Args:
                binary_image: 바이너리 이미지 (포트홀 영역이 검은색이고, 바닥/차선은 흰색이라고 가정)
                **kwargs: 추가 파라미터 (white_ratio 등 체크 함수에 전달할 값)
            
            Returns:
                bool 또는 None: 포트홀이 감지되면 True, 아니면 None
        """
        if binary_image is None:
            return None

        white_ratio = kwargs.get("white_ratio", 0.1)
        detected = check_utils.check_exist_pothole(
            binary_image,
            white_ratio_thresh=white_ratio
        )
        
        return detected
    
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

