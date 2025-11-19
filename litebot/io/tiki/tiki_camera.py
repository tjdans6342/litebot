#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
    TikiCamera 클래스
    Tiki 환경에서 카메라 이미지를 받아오는 클래스
    NVIDIA Jetson Nano의 GStreamer 파이프라인 사용
"""
import numpy as np
import cv2
from litebot.io.camera_interface import CameraInterface


class TikiCamera(CameraInterface):
    """
        Tiki 환경에서 카메라 이미지를 받아오는 클래스
        Jetson Nano의 CSI 카메라를 GStreamer로 사용
    """
    
    def __init__(self, width=640, height=480, framerate=30, flip_mode=-1):
        """
            TikiCamera 초기화
            
            Args:
                width: 이미지 너비 (기본값: 640)
                height: 이미지 높이 (기본값: 480)
                framerate: 프레임레이트 (기본값: 30)
                flip_mode: 이미지 뒤집기 모드
                    -1: 상하좌우 모두 뒤집기
                    0: 상하 뒤집기
                    1: 좌우 뒤집기
                    None: 뒤집지 않음
        """
        self.width = width
        self.height = height
        self.framerate = framerate
        self.flip_mode = flip_mode
        
        # GStreamer 파이프라인 구성
        pipeline = (
            "nvarguscamerasrc ! video/x-raw(memory:NVMM), "
            "width={}, height={}, format=NV12, framerate={}/1 ! "
            "nvvidconv ! video/x-raw, format=BGRx ! "
            "videoconvert ! video/x-raw, format=BGR ! appsink"
        ).format(width, height, framerate)
        
        # 카메라 열기
        self.cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        
        if not self.cap.isOpened():
            print("[TikiCamera] Warning: Camera Open Error. Using dummy frame.")
            self.cap = None
            self.latest_frame = None
        else:
            print("[TikiCamera] Camera opened successfully ({}x{} @ {}fps)".format(
                width, height, framerate))
            self.latest_frame = None
    
    def get_frame(self):
        """
            최신 프레임을 반환합니다.
            
            Returns:
                np.ndarray: BGR 형식의 이미지 (height, width, 3)
                또는 None (프레임이 없는 경우)
        """
        if self.cap is None:
            # 더미 프레임 반환 (테스트용)
            dummy_frame = np.zeros((self.height, self.width, 3), dtype=np.uint8)
            return dummy_frame
        
        ret, frame = self.cap.read()
        
        if not ret:
            return self.latest_frame  # 이전 프레임 반환
        
        # 이미지 뒤집기 (필요한 경우)
        if self.flip_mode is not None:
            frame = cv2.flip(frame, self.flip_mode)
        
        self.latest_frame = frame
        return frame
    
    def release(self):
        """
            카메라 리소스 해제
        """
        if self.cap is not None:
            self.cap.release()
            print("[TikiCamera] Camera released")
    
    def __del__(self):
        """
            소멸자: 카메라 리소스 해제
        """
        self.release()
