#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
    TikiCamera 클래스
    Tiki 환경에서 카메라 이미지를 받아오는 클래스
"""
import numpy as np
import cv2
from litebot.io.camera_interface import CameraInterface


class TikiCamera(CameraInterface):
    """
        Tiki 환경에서 카메라 이미지를 받아오는 클래스
    """
    
    def __init__(self):
        """
            TikiCamera 초기화
        """
        # TODO: Tiki 카메라 초기화
        pass
    
    def get_frame(self):
        """
            최신 프레임을 반환합니다.
            
            Returns:
                np.ndarray: BGR 형식의 이미지 (height, width, 3)
                또는 None (프레임이 없는 경우)
        """
        # TODO: 실제 Tiki 카메라에서 프레임 받아오기
        
        # 임시: 더미 프레임 반환 (테스트용)
        # 검은색 이미지 (640x480)
        dummy_frame = np.zeros((480, 640, 3), dtype=np.uint8)
        return dummy_frame
