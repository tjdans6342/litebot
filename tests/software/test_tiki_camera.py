#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
TikiCamera 소프트웨어 테스트
- 초기화 테스트
- 프레임 읽기 테스트
- 해상도 설정 테스트
- 리소스 해제 테스트
"""
from __future__ import print_function

import numpy as np
import time

from litebot.io.tiki.tiki_camera import TikiCamera


def test_initialization():
    """초기화 테스트"""
    print("\n=== Test 1: Initialization ===")
    
    # 기본 해상도로 초기화
    cam = TikiCamera()
    assert cam.width == 640, "기본 너비는 640이어야 함"
    assert cam.height == 480, "기본 높이는 480이어야 함"
    assert cam.framerate == 30, "기본 프레임레이트는 30이어야 함"
    print("[OK] 기본 해상도 초기화 성공 (640x480 @ 30fps)")
    
    # 커스텀 해상도로 초기화
    cam_custom = TikiCamera(width=1280, height=720, framerate=60)
    assert cam_custom.width == 1280, "커스텀 너비 설정 확인"
    assert cam_custom.height == 720, "커스텀 높이 설정 확인"
    assert cam_custom.framerate == 60, "커스텀 프레임레이트 설정 확인"
    print("[OK] 커스텀 해상도 초기화 성공 (1280x720 @ 60fps)")
    
    cam_custom.release()
    print("[PASS] Test 1: Initialization")


def test_get_frame():
    """프레임 읽기 테스트"""
    print("\n=== Test 2: get_frame ===")
    
    cam = TikiCamera()
    
    # 여러 프레임 읽기 테스트
    print("[Test] 프레임 읽기 테스트 (10회)")
    for i in range(10):
        frame = cam.get_frame()
        
        # 프레임이 None이 아니어야 함
        assert frame is not None, "프레임이 None이면 안 됨"
        
        # 프레임 형태 확인
        assert isinstance(frame, np.ndarray), "프레임은 numpy 배열이어야 함"
        assert len(frame.shape) == 3, "프레임은 3차원 배열이어야 함 (H, W, C)"
        assert frame.shape[0] == cam.height, "프레임 높이가 설정값과 일치해야 함"
        assert frame.shape[1] == cam.width, "프레임 너비가 설정값과 일치해야 함"
        assert frame.shape[2] == 3, "프레임은 3채널(BGR)이어야 함"
        assert frame.dtype == np.uint8, "프레임은 uint8 타입이어야 함"
        
        if i == 0:
            print("[OK] 프레임 형태 확인: {} (dtype: {})".format(frame.shape, frame.dtype))
    
    print("[OK] 10회 프레임 읽기 성공")
    cam.release()
    print("[PASS] Test 2: get_frame")


def test_flip_mode():
    """이미지 뒤집기 모드 테스트"""
    print("\n=== Test 3: Flip Mode ===")
    
    # 뒤집기 없음
    cam_no_flip = TikiCamera(flip_mode=None)
    frame1 = cam_no_flip.get_frame()
    assert frame1 is not None, "프레임이 None이면 안 됨"
    print("[OK] flip_mode=None 동작 확인")
    cam_no_flip.release()
    
    # 상하 뒤집기
    cam_flip_v = TikiCamera(flip_mode=0)
    frame2 = cam_flip_v.get_frame()
    assert frame2 is not None, "프레임이 None이면 안 됨"
    print("[OK] flip_mode=0 (상하 뒤집기) 동작 확인")
    cam_flip_v.release()
    
    # 좌우 뒤집기
    cam_flip_h = TikiCamera(flip_mode=1)
    frame3 = cam_flip_h.get_frame()
    assert frame3 is not None, "프레임이 None이면 안 됨"
    print("[OK] flip_mode=1 (좌우 뒤집기) 동작 확인")
    cam_flip_h.release()
    
    # 상하좌우 뒤집기
    cam_flip_both = TikiCamera(flip_mode=-1)
    frame4 = cam_flip_both.get_frame()
    assert frame4 is not None, "프레임이 None이면 안 됨"
    print("[OK] flip_mode=-1 (상하좌우 뒤집기) 동작 확인")
    cam_flip_both.release()
    
    print("[PASS] Test 3: Flip Mode")


def test_resolution_variations():
    """다양한 해상도 테스트"""
    print("\n=== Test 4: Resolution Variations ===")
    
    resolutions = [
        (320, 240, 30),
        (640, 480, 30),
        (800, 600, 30),
        (1280, 720, 60),
    ]
    
    for width, height, framerate in resolutions:
        cam = TikiCamera(width=width, height=height, framerate=framerate)
        frame = cam.get_frame()
        
        assert frame is not None, "프레임이 None이면 안 됨"
        assert frame.shape[0] == height, "높이가 설정값과 일치해야 함"
        assert frame.shape[1] == width, "너비가 설정값과 일치해야 함"
        
        print("[OK] 해상도 {}x{} @ {}fps 동작 확인".format(width, height, framerate))
        cam.release()
    
    print("[PASS] Test 4: Resolution Variations")


def test_release():
    """리소스 해제 테스트"""
    print("\n=== Test 5: Release ===")
    
    cam = TikiCamera()
    
    # 프레임 읽기
    frame = cam.get_frame()
    assert frame is not None, "프레임이 None이면 안 됨"
    
    # 리소스 해제
    cam.release()
    print("[OK] 리소스 해제 성공")
    
    # 해제 후에도 더미 프레임 반환 확인
    frame_after = cam.get_frame()
    assert frame_after is not None, "해제 후에도 더미 프레임은 반환되어야 함"
    assert frame_after.shape[0] == cam.height, "더미 프레임 높이 확인"
    assert frame_after.shape[1] == cam.width, "더미 프레임 너비 확인"
    print("[OK] 해제 후 더미 프레임 반환 확인")
    
    print("[PASS] Test 5: Release")


def test_frame_consistency():
    """프레임 일관성 테스트"""
    print("\n=== Test 6: Frame Consistency ===")
    
    cam = TikiCamera()
    
    # 연속 프레임 읽기
    frames = []
    for i in range(5):
        frame = cam.get_frame()
        frames.append(frame)
        time.sleep(0.1)  # 100ms 대기
    
    # 모든 프레임이 같은 형태여야 함
    for i, frame in enumerate(frames):
        assert frame.shape == frames[0].shape, "모든 프레임이 같은 형태여야 함"
        assert frame.dtype == frames[0].dtype, "모든 프레임이 같은 타입이어야 함"
    
    print("[OK] 연속 프레임 일관성 확인 (5회)")
    cam.release()
    print("[PASS] Test 6: Frame Consistency")


def test_camera_interface():
    """CameraInterface 인터페이스 호환성 테스트"""
    print("\n=== Test 7: CameraInterface Compatibility ===")
    
    from litebot.io.camera_interface import CameraInterface
    
    cam = TikiCamera()
    
    # CameraInterface를 구현하는지 확인
    assert isinstance(cam, CameraInterface), "TikiCamera는 CameraInterface를 구현해야 함"
    
    # 인터페이스 메서드 존재 확인
    assert hasattr(cam, 'get_frame'), "get_frame 메서드가 있어야 함"
    assert hasattr(cam, 'release'), "release 메서드가 있어야 함"
    
    # 메서드 호출 가능 확인
    frame = cam.get_frame()
    assert frame is not None, "get_frame이 동작해야 함"
    
    cam.release()
    print("[OK] CameraInterface 호환성 확인")
    print("[PASS] Test 7: CameraInterface Compatibility")


def main():
    """모든 테스트 실행"""
    print("=" * 60)
    print("TikiCamera 소프트웨어 테스트")
    print("=" * 60)
    
    try:
        test_initialization()
        test_get_frame()
        test_flip_mode()
        test_resolution_variations()
        test_release()
        test_frame_consistency()
        test_camera_interface()
        
        print("\n" + "=" * 60)
        print("[SUCCESS] All tests passed!")
        print("=" * 60)
    except AssertionError as e:
        print("\n[FAIL] Assertion failed: {}".format(e))
        import traceback
        traceback.print_exc()
    except Exception as e:
        print("\n[ERROR] Test failed: {}".format(e))
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()

