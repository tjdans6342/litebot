#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
    check_utils
    포트홀, 차선, QR 코드 등 다양한 체크 로직을 분리해서 관리하는 모듈
"""
import cv2
import numpy as np


def check_exist_pothole(binary_image,
                        top_ratio=0.3,
                        top_center_ratio=0.15,
                        side_ratio=0.1,
                        mid_range=(0.5, 0.7),
                        white_ratio_thresh=0.1):
    """
        간단한 형태 기반 조건으로 포트홀 존재 여부를 판별합니다.
    """
    if binary_image is None:
        return False

    h, w = binary_image.shape[:2]

    top_side_area1 = binary_image[:int(h * top_ratio), :int(w * side_ratio)]
    top_side_area2 = binary_image[:int(h * top_ratio), int(w * (1 - side_ratio)):]
    top_center_area = binary_image[:int(h * top_center_ratio), int(0.2 * w):int(0.8 * w)]
    mid_area = binary_image[int(h * mid_range[0]):int(h * mid_range[1]), :]

    total = top_center_area.size or 1
    white_per = float(np.sum(top_center_area == 255)) / float(total)

    return (
        white_per > white_ratio_thresh
        and mid_area.max() == 0
        and top_side_area1.max() == 0
        and top_side_area2.max() == 0
    )


def check_qrcode(image):
    """
        이미지에서 QR 코드를 감지하고 아군/적군으로 분류합니다.
        
        Args:
            image (numpy.ndarray): 
                - QR 코드를 감지할 이미지 (BGR 컬러 또는 그레이스케일)
        
        Returns:
            str 또는 None:
                - "ally"(아군), "enemy"(적군), 또는 None (QR 코드 없음 또는 분류 불가)
    """
    if image is None:
        return None
    
    # OpenCV QRCodeDetector는 컬러와 그레이스케일 모두 지원
    if len(image.shape) == 3:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    else:
        gray = image
    
    try:
        detector = cv2.QRCodeDetector()
        retval, decoded_info, points, straight_qrcode = detector.detectAndDecodeMulti(gray)
        
        if not retval or not decoded_info or len(decoded_info) == 0:
            return None
        
        # 빈 문자열이 아닌 QR 코드만 필터링
        qr_codes = [code for code in decoded_info if code and len(code) > 0]
        
        if not qr_codes:
            return None
        
        # 첫 번째 QR 코드를 분류하여 반환
        for code in qr_codes:
            result = _classify_qr_text(code)
            if result:  # ally 또는 enemy로 분류된 경우 반환
                return result
        
        # 분류할 수 없는 경우 None 반환
        return None
    
    except AttributeError:
        raise RuntimeError("cv2.QRCodeDetector가 필요합니다. OpenCV 버전을 확인하세요.")


def _classify_qr_text(qr_text):
    """
        QR 코드 텍스트를 아군/적군으로 분류합니다. (내부 함수)
        
        Args:
            qr_text (str): QR 코드에서 디코딩된 텍스트
        
        Returns:
            str: "ally"(아군), "enemy"(적군), 또는 None (분류 불가)
    """
    if not qr_text:
        return None
    
    # QR 코드 텍스트를 소문자로 변환하여 비교
    qr_lower = qr_text.lower().strip()
    
    # 아군 키워드 (필요에 따라 수정)
    ally_keywords = ["ally", "friendly", "friend", "blue", "아군", "우리편"]
    # 적군 키워드 (필요에 따라 수정)
    enemy_keywords = ["enemy", "foe", "red", "적군", "적"]
    
    if any(keyword in qr_lower for keyword in ally_keywords):
        return "ally"
    elif any(keyword in qr_lower for keyword in enemy_keywords):
        return "enemy"
    
    return None