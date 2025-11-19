#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
    ArUco 관련 유틸리티 함수
"""
import cv2
import numpy as np


def detect_largest_marker(image, aruco_dict="DICT_4X4_50", detector_params=None):
    """
        이미지에서 가장 큰 ArUco 마커를 탐지합니다.

        Args:
            image (numpy.ndarray):
                - BGR 컬러 또는 단일 채널(그레이스케일) 이미지.
                - ROS 카메라 프레임, 동영상 프레임 등 원본 이미지를 그대로 전달합니다.
            aruco_dict (str):
                - 사용할 ArUco 딕셔너리 이름.
                - 기본값 "DICT_4X4_50"이며, OpenCV에서 지원하는 다른 딕셔너리 이름으로 교체 가능.
            detector_params (cv2.aruco.DetectorParameters, optional):
                - 사용자 정의 아루코 검출 파라미터. 지정하지 않으면 OpenCV 기본 파라미터 사용.

        Returns:
            dict 또는 None:
                - {"id": int, "corners": list[list[float, float]], "center": (float, float)}
                    * id: 검출된 마커의 정수 ID
                    * corners: 네 꼭짓점 좌표 [[x1, y1], ..., [x4, y4]]
                    * center: (cx, cy) 형태의 마커 중심 좌표
                - 마커가 없으면 None
    """
    if image is None:
        return None

    if len(image.shape) == 3:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    else:
        gray = image

    try:
        aruco_module = cv2.aruco
    except AttributeError:
        raise RuntimeError("cv2.aruco 모듈이 필요합니다. opencv-contrib-python 설치 여부를 확인하세요.")

    dictionary = getattr(aruco_module, aruco_dict, aruco_module.DICT_4X4_50)
    
    # OpenCV 버전 호환성 처리
    # OpenCV 4.7.0+ 에서는 ArucoDetector 사용
    # OpenCV 4.x 에서는 getPredefinedDictionary 사용
    # OpenCV 3.x 에서는 Dictionary_get 사용
    try:
        # OpenCV 4.7.0+ 방식 시도
        aruco_dict_obj = aruco_module.getPredefinedDictionary(dictionary)
        if detector_params is None:
            parameters = aruco_module.DetectorParameters()
        else:
            parameters = detector_params
        
        try:
            # ArucoDetector 사용 (OpenCV 4.7.0+)
            detector = aruco_module.ArucoDetector(aruco_dict_obj, parameters)
            corners, ids, _ = detector.detectMarkers(gray)
        except (AttributeError, TypeError):
            # detectMarkers 직접 사용 (OpenCV 4.x)
            corners, ids, _ = aruco_module.detectMarkers(gray, aruco_dict_obj, parameters=parameters)
    except (AttributeError, TypeError):
        # OpenCV 3.x 방식
        aruco_dict_obj = aruco_module.Dictionary_get(dictionary)
        if detector_params is None:
            parameters = aruco_module.DetectorParameters_create()
        else:
            parameters = detector_params
        corners, ids, _ = aruco_module.detectMarkers(gray, aruco_dict_obj, parameters=parameters)
    if ids is None or len(ids) == 0:
        return None

    areas = []
    for marker_corners in corners:
        pts = marker_corners[0]
        area = cv2.contourArea(pts.astype(np.float32))
        areas.append(area)

    max_idx = int(np.argmax(areas))
    marker_corners = corners[max_idx][0]
    center_x = float(np.mean(marker_corners[:, 0]))
    center_y = float(np.mean(marker_corners[:, 1]))

    return {
        "id": int(ids[max_idx][0]),
        "corners": marker_corners.tolist(),
        "center": (center_x, center_y)
    }