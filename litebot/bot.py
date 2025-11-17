#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
LiteBot 클래스
자율주행 로봇의 메인 클래스로 모든 컴포넌트를 통합합니다.
"""
from litebot.io.ros.ros_camera import ROSCamera
from litebot.io.tiki.tiki_camera import TikiCamera
from litebot.io.ros.ros_controller import ROSController
from litebot.io.tiki.tiki_controller import TikiController
from litebot.processing.image_processor import ImageProcessor
from litebot.core.observer import Observer
from litebot.core.trigger_manager import TriggerManager
from litebot.core.fire_detector import FireDetector
from litebot.action.action_executor import ActionExecutor


class LiteBot:
    """
    자율주행 로봇의 메인 클래스
    Camera, ImageProcessor, Observer, TriggerManager, ActionExecutor를 통합합니다.
    """
    
    def __init__(self, mode="ros"):
        """
        LiteBot 초기화
        
        Args:
            mode: "ros" 또는 "tiki" - 사용할 하드웨어 모드
        """
        # mode 검증 및 저장
        if mode not in ["ros", "tiki"]:
            raise ValueError("Unknown mode: {}. Use 'ros' or 'tiki'".format(mode))
        self.mode = mode
        
        # Controller 초기화
        self.controller = ROSController() if mode == "ros" else TikiController()
        
        # Camera 초기화
        self.camera = ROSCamera() if mode == "ros" else TikiCamera()
        
        # ImageProcessor 초기화
        self.image_processor = ImageProcessor()
        
        # Observer 초기화
        self.observer = Observer()
        
        # FireDetector 초기화 (파일 기반)
        self.fire_detector = FireDetector()
        
        # TriggerManager 초기화
        self.trigger_manager = TriggerManager()
        
        # ActionExecutor 초기화
        self.action_executor = ActionExecutor(self.controller)
        
        # Recording (나중에 구현)
        # self.recording = Recording()
        
        # 상태 변수
        self.frame = None
        self.images = None
    
    def step(self):
        """
        한 스텝의 주행 사이클을 수행합니다.
        
        Pipeline:
        1. 프레임 캡처
        2. 이미지 처리
        3. 감지 수행
        4. 트리거 매니저가 적절한 액션을 반환
        5. 액션 실행 (ActionExecutor가 리소스 타입별로 실행 제어)
        
        Note:
            - 매 프레임마다 호출되며, 비동기 액션이 실행 중이어도 계속 진행됩니다.
            - ActionExecutor가 리소스 타입별로 실행 제어를 담당합니다.
            - 같은 리소스 타입의 액션이 실행 중이면 새 액션은 무시됩니다.
            - 리소스와 독립적인 액션(capture, qr_command)은 항상 실행됩니다.
        
        Returns:
            tuple: (observations, action, source)
                - observations: 관찰 결과 딕셔너리
                - action: 실행된 액션 또는 None
                - source: 우세 트리거명 또는 None
        """
        # 1. 프레임 캡처
        self.frame = self.camera.get_frame()
        
        # 프레임이 없으면 처리 중단
        if self.frame is None:
            return None, None, None
        
        # 2. 이미지 처리
        self.images = self.image_processor.get_images(self.frame)
        
        # images가 None이면 빈 딕셔너리로 처리
        if self.images is None:
            self.images = {}
        
        # 3. 화재 감지 파일 확인 (건물 번호 수신)
        self.fire_detector.set()
        
        # 4. 감지 수행
        observations = {
            "lane": self.observer.observe_lines(self.images["hough"]),
            "aruco": self.observer.observe_aruco(self.images["original"]),
            "pothole": self.observer.observe_pothole(self.images["binary"]),
            "qr_codes": self.observer.observe_qr_codes(self.images["original"]),
            # 필요한 경우 다른 감지 추가
        }
        
        # 5. 트리거 매니저가 적절한 액션과 우세 트리거명을 반환
        action, source = self.trigger_manager.step(observations)
        
        # 5. 액션 실행
        if action:
            self.action_executor.execute(action)
        
        return observations, action, source

