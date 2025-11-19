#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
이미지 처리 클래스
카메라에서 받은 프레임을 다양한 방식으로 처리하여 반환합니다.
"""
import cv2
import numpy as np
from litebot.utils import image_utils


class ImageProcessor:
    """
    이미지 처리 파이프라인을 관리하는 클래스
    ROI, BEV, color_filter, Hough Line Transform 등을 수행합니다.
    """
    
    def __init__(self):
        """
        ImageProcessor 초기화
        TODO: 나중에 config에서 파라미터를 가져오도록 수정
        """
        # 기본 파라미터 (나중에 config로 이동)
        self._init_default_params()
    
    def _init_default_params(self):
        """
        기본 파라미터 초기화
        TODO: 나중에 configs/lane_config.py에서 가져오도록 수정
        """

        # --- BEV 변환 파라미터 ---
        self.bev_normalized = True
        self.roi_top = 0.75
        self.roi_bottom = 0.0
        self.roi_margin = 0.1
        self.bev_dst_size = None  # (width, height) 또는 None (자동 계산)

        # --- Color filter 파라미터 (HLS 범위) ---
        # color_filter 함수는 HLS 기반이므로 HLS 범위 사용
        # self.hls_range = [[(70, 110, 20), (180, 255, 255)]]
        self.hls_range = [[(66, 180, 0), (180, 255, 255)]]

        # --- 이미지 전처리 파라미터 ---
        self.binary_threshold = (20, 255)  # threshold 임계값 (min, max)

        # --- Hough Line 파라미터 ---
        self.hough_threshold = 50  # Hough accumulator 임계값 (min_votes)
        self.hough_slope_threshold = 10  # 수평선 필터링 임계값 (degree)
        self.hough_min_line_len = 50  # 최소 선분 길이
        self.hough_max_line_gap = 10  # 선분 사이의 최대 간격
    
    def get_images(self, frame):
        """
        프레임을 받아서 다양한 처리된 이미지들을 반환
        
        Pipeline:
            Original → BEV → color_filter → Gray Scale → GaussianBlur 
            → threshold → largest_component (8방향 연결성 분석) → Canny → get_hough_image
        
        Args:
            frame: 카메라에서 받은 원본 프레임
        
        Returns:
            dict: 처리된 이미지들의 딕셔너리
                - "original": 원본 이미지
                - "bev": BEV 변환된 이미지
                - "filtered": Color filter 적용된 이미지
                - "gray": 그레이스케일 이미지
                - "blur": GaussianBlur 적용된 이미지
                - "binary": 바이너리 이미지
                - "canny": Canny edge detection 이미지
                - "hough": Hough Line이 그려진 이미지
        """
        if frame is None:
            return {"original": None}
        
        height, width = frame.shape[:2]
        
        # 1. BEV 처리
        bev_img, bev_minv = image_utils.to_bev(
            frame,
            top=self.roi_top,
            bottom=self.roi_bottom,
            margin=self.roi_margin,
            dst_size=self.bev_dst_size,
            normalized=self.bev_normalized
        )
        
        # 2. Color filter 처리
        filtered_img = image_utils.color_filter(bev_img, hls_range=self.hls_range, inverse=False)
        
        # 3. Gray Scale 변환
        gray_img = cv2.cvtColor(filtered_img, cv2.COLOR_BGR2GRAY)
        
        # 4. GaussianBlur 적용 (고정값: (7, 7), sigma=5)
        blur_img = cv2.GaussianBlur(gray_img, (7, 7), 5)
        
        # 5. Threshold 적용
        _, binary_img = cv2.threshold(
            blur_img, 
            self.binary_threshold[0], 
            self.binary_threshold[1], 
            cv2.THRESH_BINARY
        )
        
        # 5-1. 연결된 컴포넌트 분석: 가장 큰 집합(도로)만 남기기
        component_binary_img, component_count, largest_size = image_utils.get_largest_component(
            binary_img, 
            connectivity=8
        )
        
        # 6. Canny edge detection (고정값: low=10, high=100)
        canny_img = cv2.Canny(component_binary_img, 10, 100)
        
        # 7. Hough Line 처리
        hough_img = image_utils.get_hough_image(
            canny_img,
            slope_threshold=self.hough_slope_threshold,
            min_votes=self.hough_threshold,
            min_line_len=self.hough_min_line_len,
            max_line_gap=self.hough_max_line_gap
        )
        
        return {
            "original": frame.copy(),
            "bev": bev_img,
            "filtered": filtered_img,
            "gray": gray_img,
            "blur": blur_img,
            "binary": binary_img,
            "canny": canny_img,
            "hough": hough_img,
            "component_binary_img": component_binary_img
        }

