#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
    TikiCamera 클래스
    Tiki 환경에서 카메라 이미지를 받아오는 클래스
    NVIDIA Jetson Nano의 GStreamer 파이프라인 사용
"""
import time
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
        
        # GStreamer 파이프라인 구성 (예제 코드와 동일한 형식)
        pipeline = (
            "nvarguscamerasrc ! video/x-raw(memory:NVMM), "
            "width={}, height={}, format=NV12, framerate={}/1 ! "
            "nvvidconv ! video/x-raw, format=BGRx ! "
            "videoconvert ! video/x-raw, format=BGR ! appsink"
        ).format(width, height, framerate)
        
        # 카메라 열기 (재시도 로직 포함)
        self.cap = None
        max_retries = 3
        retry_delay = 0.5
        
        for attempt in range(max_retries):
            self.cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
            
            if self.cap.isOpened():
                # 카메라 초기화를 위한 짧은 대기 시간
                time.sleep(0.1)
                
                # 초기 프레임 읽기 시도 (카메라가 실제로 동작하는지 확인)
                ret, test_frame = self.cap.read()
                if ret and test_frame is not None:
                    print("[TikiCamera] Camera opened successfully ({}x{} @ {}fps)".format(
                        width, height, framerate))
                    self.latest_frame = None
                    break
                else:
                    # 카메라는 열렸지만 프레임을 읽을 수 없음
                    self.cap.release()
                    self.cap = None
            
            if attempt < max_retries - 1:
                print("[TikiCamera] Camera open attempt {} failed, retrying...".format(attempt + 1))
                time.sleep(retry_delay)
        
        if self.cap is None or not self.cap.isOpened():
            print("[TikiCamera] Warning: Camera Open Error after {} attempts. Using dummy frame.".format(max_retries))
            if self.cap is not None:
                self.cap.release()
            self.cap = None
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
        
        if not ret or frame is None:
            # 이전 프레임이 있으면 반환, 없으면 더미 프레임 반환
            if self.latest_frame is not None:
                return self.latest_frame
            else:
                # 초기 프레임이 없을 때 더미 프레임 반환
                dummy_frame = np.zeros((self.height, self.width, 3), dtype=np.uint8)
                return dummy_frame
        
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
            self.cap = None
            print("[TikiCamera] Camera released")
            # Argus가 세션을 정리할 시간 확보
            time.sleep(0.5)
    
    def __del__(self):
        """
            소멸자: 카메라 리소스 해제
        """
        self.release()
