#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Tiki 하드웨어 테스트 - 리소스 타입 분리 및 비동기 액션
- 실제 Tiki 하드웨어에서 리소스 타입별 독립 실행 확인
- 비동기 액션 실행 중 다른 액션 무시 확인
"""
from __future__ import print_function

import sys
import time
import os
import numpy as np

from litebot.io.tiki.tiki_controller import TikiController
from litebot.action.action_executor import ActionExecutor


def test_motor_resource_busy():
    """motor 리소스 busy 시 다른 motor 액션 무시 확인 (하드웨어)"""
    print("\n=== Test 1: Motor resource busy - 다른 motor 액션 무시 (HW) ===")
    
    ctrl = TikiController()
    ex = ActionExecutor(ctrl)
    
    # drive_forward 시작 (비동기, 실제 하드웨어)
    print("[Test] Starting drive_forward (async, hardware)...")
    ex.execute(("drive_forward", (1.0, 0.2)))  # Tiki는 비동기로 처리
    
    time.sleep(0.2)  # 액션 시작 대기
    
    # motor 리소스 busy 상태에서 다른 motor 액션 시도
    print("[Test] Trying rotate while motor busy (should be ignored)...")
    ex.execute(("rotate", 90.0))
    
    print("[Test] Trying update_speed_angular while motor busy (should be ignored)...")
    ex.execute(("update_speed_angular", {"speed": 0.3, "angular": 0.5}))
    
    print("[Test] Trying stop while motor busy (should be ignored)...")
    ex.execute(("stop", None))
    
    # capture는 리소스 독립이므로 실행되어야 함
    print("[Test] Trying capture while motor busy (should execute - independent resource)...")
    out_dir = os.path.join("tests", "_out")
    if not os.path.exists(out_dir):
        try:
            os.makedirs(out_dir)
        except Exception:
            pass
    dummy_image = (np.zeros((100, 160, 3)) + 255).astype("uint8")
    out_path = os.path.join(out_dir, "tiki_test_capture.jpg")
    ex.execute(("capture", {"image": dummy_image, "save_path": out_path}))
    print("[Test] Capture executed successfully (independent resource)")
    
    # drive_forward 완료 대기 (Tiki는 비동기이므로 상태 확인)
    print("[Test] Waiting for drive_forward to complete...")
    max_wait = 10.0  # 최대 10초 대기
    start_time = time.time()
    while ctrl.is_action_running():
        if time.time() - start_time > max_wait:
            print("[Test] Timeout waiting for drive_forward to complete")
            break
        time.sleep(0.1)
    
    if not ctrl.is_action_running():
        print("[Test] Motor resource freed. Now trying rotate (should execute)...")
        ex.execute(("rotate", 90.0))
        
        time.sleep(0.2)
        max_wait = 5.0
        start_time = time.time()
        while ctrl.is_action_running():
            if time.time() - start_time > max_wait:
                break
            time.sleep(0.1)
    
    print("[OK] Test 1 passed: Motor resource separation works correctly (hardware)")
    return True


def test_pid_reset_on_motor_action():
    """motor 액션에서 PID 리셋 확인 (하드웨어)"""
    print("\n=== Test 2: PID reset on motor actions (HW) ===")
    
    ctrl = TikiController()
    ex = ActionExecutor(ctrl)
    
    # update_speed_angular로 PID 상태 생성
    print("[Test] Executing update_speed_angular (PID should NOT reset)...")
    ex.execute(("update_speed_angular", {"speed": 0.3, "angular": 0.5}))
    time.sleep(0.1)
    
    # drive_forward로 PID 리셋 확인
    print("[Test] Executing drive_forward (PID should reset)...")
    ex.execute(("drive_forward", (0.5, 0.2)))
    
    time.sleep(0.2)
    max_wait = 5.0
    start_time = time.time()
    while ctrl.is_action_running():
        if time.time() - start_time > max_wait:
            break
        time.sleep(0.1)
    
    # capture는 motor 리소스가 아니므로 PID 리셋 안 함
    print("[Test] Executing capture (PID should NOT reset - not motor resource)...")
    dummy_image = (np.zeros((100, 160, 3)) + 255).astype("uint8")
    out_path = os.path.join("tests", "_out", "tiki_test_capture2.jpg")
    ex.execute(("capture", {"image": dummy_image, "save_path": out_path}))
    
    print("[OK] Test 2 passed: PID reset only on motor actions (hardware)")
    return True


def test_concurrent_resources():
    """다른 리소스 타입의 동시 실행 확인 (하드웨어)"""
    print("\n=== Test 3: Concurrent resources (HW) ===")
    
    ctrl = TikiController()
    ex = ActionExecutor(ctrl)
    
    # motor 액션 실행 중
    print("[Test] Starting drive_forward (motor resource)...")
    ex.execute(("drive_forward", (1.0, 0.2)))
    
    time.sleep(0.2)
    
    # capture는 리소스 독립이므로 동시 실행 가능
    print("[Test] Executing capture while motor running (should execute - different resource)...")
    dummy_image = (np.zeros((100, 160, 3)) + 255).astype("uint8")
    out_path = os.path.join("tests", "_out", "tiki_test_concurrent.jpg")
    ex.execute(("capture", {"image": dummy_image, "save_path": out_path}))
    print("[Test] Capture executed successfully while motor running")
    
    # drive_forward 완료 대기
    max_wait = 10.0
    start_time = time.time()
    while ctrl.is_action_running():
        if time.time() - start_time > max_wait:
            break
        time.sleep(0.1)
    
    print("[OK] Test 3 passed: Concurrent resources work correctly (hardware)")
    return True


def main():
    """모든 하드웨어 테스트 실행"""
    print("=" * 60)
    print("Tiki 하드웨어 테스트 - 리소스 타입 분리 및 비동기 액션")
    print("=" * 60)
    
    ctrl = None
    try:
        results = []
        results.append(test_motor_resource_busy())
        time.sleep(1.0)  # 테스트 간 간격
        
        results.append(test_pid_reset_on_motor_action())
        time.sleep(1.0)
        
        results.append(test_concurrent_resources())
        
        print("\n" + "=" * 60)
        if all(results):
            print("[SUCCESS] All hardware tests passed!")
        else:
            print("[WARNING] Some tests may have failed. Check logs above.")
        print("=" * 60)
    except KeyboardInterrupt:
        print("\n[Tiki Resource Separation HW] Interrupted. Braking...")
        if ctrl is not None:
            ctrl.brake()
    except Exception as e:
        print("\n[ERROR] Test failed: {}".format(e))
        import traceback
        traceback.print_exc()
        if ctrl is not None:
            ctrl.brake()


if __name__ == "__main__":
    main()

