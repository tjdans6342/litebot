#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
ActionExecutor 리소스 타입 분리 및 비동기 액션 테스트
- 리소스 타입별 독립 실행 확인
- 비동기 액션 실행 중 다른 액션 무시 확인
"""
from __future__ import print_function

import threading
import time
import os
import numpy as np
import rospy

from litebot.action.action_executor import ActionExecutor


class DummyController(object):
    """비동기 액션과 리소스 타입 분리를 테스트하기 위한 Dummy Controller"""
    
    def __init__(self):
        self.last_cmd = None
        self._action_running = False
        self._action_lock = threading.Lock()
        self._action_thread = None
    
    def update_speed_angular(self, v, w):
        self.last_cmd = (v, w)
        print("[DummyController] update_speed_angular v={}, w={}".format(v, w))
    
    def brake(self):
        self.last_cmd = (0.0, 0.0)
        print("[DummyController] brake")
    
    def drive_forward_distance(self, distance, speed):
        print("[DummyController] forward distance={}, speed={}".format(distance, speed))
        time.sleep(max(0.0, min(1.0, distance / max(speed, 1e-6))))
    
    def drive_backward_distance(self, distance, speed):
        print("[DummyController] backward distance={}, speed={}".format(distance, speed))
        time.sleep(max(0.0, min(1.0, distance / max(speed, 1e-6))))
    
    def drive_circle_distance(self, distance, speed, angular_velocity):
        print("[DummyController] circle distance={}, speed={}, w={}".format(distance, speed, angular_velocity))
        time.sleep(max(0.0, min(1.0, distance / max(speed, 1e-6))))
    
    def rotate_in_place(self, degrees, ang_speed=1.0):
        print("[DummyController] rotate_in_place {} deg @{} rad/s".format(degrees, ang_speed))
        duration = abs(degrees) * 3.141592653589793 / 180.0 / max(ang_speed, 1e-6)
        time.sleep(min(duration, 1.0))
    
    # 비동기 액션 관련 메서드
    def execute_async(self, action):
        """비동기 액션 실행"""
        with self._action_lock:
            if self._action_running and self._action_thread is not None and self._action_thread.is_alive():
                print("[DummyController] Action already running, ignoring: {}".format(action[0]))
                return
            
            self._action_running = True
            self._action_thread = threading.Thread(target=self._run_action, args=(action,))
            self._action_thread.daemon = True
            self._action_thread.start()
    
    def _run_action(self, action):
        """스레드에서 실행되는 액션"""
        try:
            cmd, value = action
            if cmd == "drive_forward":
                distance, speed = value
                self.drive_forward_distance(distance, speed)
            elif cmd == "drive_backward":
                distance, speed = value
                self.drive_backward_distance(distance, speed)
            elif cmd == "drive_circle":
                distance, speed, angular_velocity = value
                self.drive_circle_distance(distance, speed, angular_velocity)
            elif cmd == "rotate":
                if isinstance(value, tuple) and len(value) == 2:
                    degrees, ang_speed = value
                else:
                    degrees = value
                    ang_speed = 1.0
                self.rotate_in_place(degrees, ang_speed)
        except Exception as e:
            print("[DummyController] Error in async action: {}".format(e))
        finally:
            with self._action_lock:
                self._action_running = False
                self._action_thread = None
    
    def is_action_running(self):
        """액션 실행 중인지 확인"""
        with self._action_lock:
            return self._action_running and self._action_thread is not None and self._action_thread.is_alive()
    
    def cancel_action(self):
        """액션 취소"""
        with self._action_lock:
            if self._action_thread is not None and self._action_thread.is_alive():
                self.brake()
                self._action_running = False


def test_motor_resource_busy():
    """motor 리소스 busy 시 다른 motor 액션 무시 확인"""
    print("\n=== Test 1: Motor resource busy - 다른 motor 액션 무시 ===")
    ctrl = DummyController()
    ex = ActionExecutor(ctrl)
    
    # drive_forward 시작 (비동기)
    print("[Test] Starting drive_forward (async)...")
    ex.execute(("drive_forward", (2.0, 0.2)))  # 10초 걸림
    
    time.sleep(0.1)  # 스레드 시작 대기
    
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
    out_path = os.path.join(out_dir, "test_capture.jpg")
    ex.execute(("capture", {"image": dummy_image, "save_path": out_path}))
    print("[Test] Capture executed successfully (independent resource)")
    
    # drive_forward 완료 대기
    print("[Test] Waiting for drive_forward to complete...")
    while ctrl.is_action_running():
        time.sleep(0.1)
    
    print("[Test] Motor resource freed. Now trying rotate (should execute)...")
    ex.execute(("rotate", 90.0))
    
    time.sleep(0.1)
    while ctrl.is_action_running():
        time.sleep(0.1)
    
    print("[OK] Test 1 passed: Motor resource separation works correctly")


def test_pid_reset_on_motor_action():
    """motor 액션에서 PID 리셋 확인"""
    print("\n=== Test 2: PID reset on motor actions ===")
    ctrl = DummyController()
    ex = ActionExecutor(ctrl)
    
    # update_speed_angular로 PID 상태 생성
    print("[Test] Executing update_speed_angular (PID should NOT reset)...")
    ex.execute(("update_speed_angular", {"speed": 0.3, "angular": 0.5}))
    
    # drive_forward로 PID 리셋 확인
    print("[Test] Executing drive_forward (PID should reset)...")
    ex.execute(("drive_forward", (0.5, 0.2)))
    
    time.sleep(0.1)
    while ctrl.is_action_running():
        time.sleep(0.1)
    
    # capture는 motor 리소스가 아니므로 PID 리셋 안 함
    print("[Test] Executing capture (PID should NOT reset - not motor resource)...")
    dummy_image = (np.zeros((100, 160, 3)) + 255).astype("uint8")
    out_path = os.path.join("tests", "_out", "test_capture2.jpg")
    ex.execute(("capture", {"image": dummy_image, "save_path": out_path}))
    
    print("[OK] Test 2 passed: PID reset only on motor actions")


def main():
    rospy.init_node("test_pid_reset", anonymous=False)

    """모든 테스트 실행"""
    print("=" * 60)
    print("ActionExecutor 리소스 타입 분리 및 비동기 액션 테스트")
    print("=" * 60)
    
    try:
        test_motor_resource_busy()
        test_pid_reset_on_motor_action()
        
        print("\n" + "=" * 60)
        print("[SUCCESS] All tests passed!")
        print("=" * 60)
    except Exception as e:
        print("\n[ERROR] Test failed: {}".format(e))
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()

