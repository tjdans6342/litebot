#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Camera Interface
카메라 클래스들이 구현해야 하는 인터페이스
"""
import numpy as np


class CameraInterface:
    """
    카메라 인터페이스
    모든 카메라 클래스는 이 인터페이스를 구현해야 합니다.
    """
    
    def get_frame(self):
        """
        최신 프레임을 반환합니다.
        
        Returns:
            np.ndarray: BGR 형식의 이미지 (height, width, 3)
            또는 None (프레임이 없는 경우)
        """
        raise NotImplementedError("Subclass must implement get_frame() method")
