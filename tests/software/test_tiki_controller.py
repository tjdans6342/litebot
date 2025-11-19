#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
TikiController 소프트웨어 테스트
- 모든 메서드 동작 확인
- 비동기 액션 실행 확인
- 모터 모드 설정 확인
"""
from __future__ import print_function

import time
import threading

from litebot.io.tiki.tiki_controller import TikiController, TIKI_AVAILABLE

_REGISTERED_CONTROLLERS = []


def _register_controller(ctrl):
    """
    생성된 컨트롤러를 등록하여 종료 시 brake 호출
    """
    if ctrl is not None and ctrl not in _REGISTERED_CONTROLLERS:
        _REGISTERED_CONTROLLERS.append(ctrl)


def _require_hardware(test_name):
    """
    하드웨어(TikiMini)가 없는 경우 테스트를 건너뛰기 위한 헬퍼
    """
    if not TIKI_AVAILABLE:
        print("[SKIP] {} - tiki.mini 모듈이 없어 stub 모드로 실행 중입니다.".format(test_name))
        return None
    
    ctrl = TikiController()
    _register_controller(ctrl)
    if ctrl.tiki is None:
        print("[SKIP] {} - TikiMini 인스턴스를 생성하지 못했습니다.".format(test_name))
        return None
    
    return ctrl


def _cleanup_controllers():
    """
    테스트 종료 시 등록된 컨트롤러 브레이크
    """
    if not _REGISTERED_CONTROLLERS:
        return
    print("\n[Cleanup] Braking {} controller(s)...".format(len(_REGISTERED_CONTROLLERS)))
    for ctrl in _REGISTERED_CONTROLLERS:
        try:
            ctrl.brake()
        except Exception as exc:
            print("[Cleanup] brake failed: {}".format(exc))


def test_initialization():
    """초기화 테스트"""
    print("\n=== Test 1: Initialization ===")
    
    # PID 모드로 초기화
    ctrl_pid = TikiController(motor_mode='PID')
    _register_controller(ctrl_pid)
    assert ctrl_pid.motor_mode == 'PID', "PID 모드 초기화 실패"
    assert ctrl_pid.current_speed == 0.0, "초기 속도는 0이어야 함"
    print("[OK] PID 모드 초기화 성공")
    
    # PWM 모드로 초기화
    ctrl_pwm = TikiController(motor_mode='PWM')
    _register_controller(ctrl_pwm)
    assert ctrl_pwm.motor_mode == 'PWM', "PWM 모드 초기화 실패"
    print("[OK] PWM 모드 초기화 성공")
    
    print("[PASS] Test 1: Initialization")


def test_brake():
    """정지 테스트"""
    print("\n=== Test 2: Brake ===")
    
    ctrl = TikiController()
    _register_controller(ctrl)
    
    # 속도 설정 후 정지
    ctrl.update_speed_angular(0.5, 0.0)
    assert ctrl.current_speed != 0.0, "속도가 설정되어야 함"
    
    ctrl.brake()
    assert ctrl.current_speed == 0.0, "정지 후 속도는 0이어야 함"
    
    print("[PASS] Test 2: Brake")


def test_update_speed_angular():
    """선속도/각속도 업데이트 테스트"""
    print("\n=== Test 3: update_speed_angular ===")
    
    ctrl = TikiController()
    _register_controller(ctrl)
    
    # 다양한 속도 조합 테스트
    test_cases = [
        (0.5, 0.0),    # 직진
        (0.3, 0.5),    # 좌회전
        (0.3, -0.5),   # 우회전
        (0.0, 1.0),    # 제자리 회전
    ]
    
    for linear_x, angular_z in test_cases:
        ctrl.update_speed_angular(linear_x, angular_z)
        assert ctrl.current_speed == linear_x, "속도가 올바르게 설정되어야 함"
        print("[OK] v={}, w={} 설정 성공".format(linear_x, angular_z))
    
    print("[PASS] Test 3: update_speed_angular")


# def test_drive_forward_distance():
#     """전진 거리 테스트"""
#     print("\n=== Test 4: drive_forward_distance ===")
    
#     ctrl = _require_hardware("Test 4: drive_forward_distance")
#     if ctrl is None:
#         return
    
#     # 짧은 거리 테스트
#     print("[Test] 0.1m 전진 @ 0.2 m/s")
#     start_time = time.time()
#     ctrl.drive_forward_distance(1.0, 0.1)
#     duration = time.time() - start_time
    
#     assert ctrl.current_speed == 0.0, "전진 완료 후 속도는 0이어야 함"
#     print("[OK] 전진 완료 (소요 시간: {:.2f}초)".format(duration))
    
#     print("[PASS] Test 4: drive_forward_distance")


def test_drive_forward_distance():
    """전진 거리 테스트"""
    print("\n=== Test 4: drive_forward_distance ===")

    ctrl = _require_hardware("Test 4: drive_forward_distance")
    if ctrl is None:
        return

    # 짧은 거리 테스트
    print("[Test] 0.1m 전진 @ 0.2 m/s")

    start_time = time.time()
    ctrl.drive_forward_distance(0.1, 0.2)   # 거리(m), 속도(v)
    end_time = time.time()

    duration = end_time - start_time

    assert ctrl.current_speed == 0.0, "전진 완료 후 속도는 0이어야 함"

    print("[OK] 전진 완료")
    print("[TIME] drive_forward_distance 실행 시간: {:.3f} 초".format(duration))

    print("[PASS] Test 4: drive_forward_distance")


def test_drive_backward_distance():
    """후진 거리 테스트"""
    print("\n=== Test 5: drive_backward_distance ===")
    
    ctrl = _require_hardware("Test 5: drive_backward_distance")
    if ctrl is None:
        return
    
    # 짧은 거리 테스트
    print("[Test] 0.1m 후진 @ 0.2 m/s")
    start_time = time.time()
    ctrl.drive_backward_distance(0.1, 0.2)
    duration = time.time() - start_time
    
    assert ctrl.current_speed == 0.0, "후진 완료 후 속도는 0이어야 함"
    print("[OK] 후진 완료 (소요 시간: {:.2f}초)".format(duration))
    
    print("[PASS] Test 5: drive_backward_distance")


def test_drive_circle_distance():
    """원형 주행 테스트"""
    print("\n=== Test 6: drive_circle_distance ===")
    
    ctrl = _require_hardware("Test 6: drive_circle_distance")
    if ctrl is None:
        return
    
    # 좌회전 원형 주행
    print("[Test] 좌회전 원형 주행 (0.5m, v=0.3, w=1.0)")
    start_time = time.time()
    ctrl.drive_circle_distance(0.5, 0.3, 1.0)
    duration = time.time() - start_time
    
    assert ctrl.current_speed == 0.0, "원형 주행 완료 후 속도는 0이어야 함"
    print("[OK] 좌회전 원형 주행 완료 (소요 시간: {:.2f}초)".format(duration))
    
    # 우회전 원형 주행
    print("[Test] 우회전 원형 주행 (0.5m, v=0.3, w=-1.0)")
    start_time = time.time()
    ctrl.drive_circle_distance(0.5, 0.3, -1.0)
    duration = time.time() - start_time
    
    assert ctrl.current_speed == 0.0, "원형 주행 완료 후 속도는 0이어야 함"
    print("[OK] 우회전 원형 주행 완료 (소요 시간: {:.2f}초)".format(duration))
    
    print("[PASS] Test 6: drive_circle_distance")


def test_rotate_in_place():
    """제자리 회전 테스트"""
    print("\n=== Test 7: rotate_in_place ===")
    
    ctrl = _require_hardware("Test 7: rotate_in_place")
    if ctrl is None:
        return
    
    # 좌회전 테스트
    print("[Test] 좌회전 90도 @ 1.0 rad/s")
    start_time = time.time()
    ctrl.rotate_in_place(90.0, ang_speed=1.0)
    duration = time.time() - start_time
    
    assert ctrl.current_speed == 0.0, "회전 완료 후 속도는 0이어야 함"
    print("[OK] 좌회전 완료 (소요 시간: {:.2f}초)".format(duration))
    
    # 우회전 테스트
    print("[Test] 우회전 -90도 @ 1.0 rad/s")
    start_time = time.time()
    ctrl.rotate_in_place(-90.0, ang_speed=1.0)
    duration = time.time() - start_time
    
    assert ctrl.current_speed == 0.0, "회전 완료 후 속도는 0이어야 함"
    print("[OK] 우회전 완료 (소요 시간: {:.2f}초)".format(duration))
    
    print("[PASS] Test 7: rotate_in_place")


def test_async_action():
    """비동기 액션 테스트"""
    print("\n=== Test 8: Async Action ===")
    
    ctrl = _require_hardware("Test 8: Async Action")
    if ctrl is None:
        return
    
    # 비동기 전진 실행
    print("[Test] 비동기 전진 시작")
    ctrl.execute_async(("drive_forward", (0.5, 0.2)))
    
    # 액션이 실행 중인지 확인
    time.sleep(0.1)
    assert ctrl.is_action_running(), "액션이 실행 중이어야 함"
    print("[OK] 액션 실행 중 확인")
    
    # 실행 중인 액션 취소
    print("[Test] 액션 취소")
    ctrl.cancel_action()
    time.sleep(0.1)
    assert not ctrl.is_action_running(), "액션이 취소되어야 함"
    print("[OK] 액션 취소 성공")
    
    # 새로운 액션 실행 중에 다른 액션 무시 확인
    print("[Test] 실행 중인 액션 무시 확인")
    ctrl.execute_async(("drive_forward", (1.0, 0.2)))
    time.sleep(0.1)
    
    # 다른 액션 시도 (무시되어야 함)
    ctrl.execute_async(("rotate", 90.0))
    time.sleep(0.1)
    
    # 첫 번째 액션이 여전히 실행 중이어야 함
    assert ctrl.is_action_running(), "첫 번째 액션이 계속 실행 중이어야 함"
    print("[OK] 실행 중인 액션 무시 확인")
    
    # 액션 완료 대기
    while ctrl.is_action_running():
        time.sleep(0.1)
    
    print("[PASS] Test 8: Async Action")


def test_async_actions():
    """다양한 비동기 액션 테스트"""
    print("\n=== Test 9: Various Async Actions ===")
    
    ctrl = _require_hardware("Test 9: Various Async Actions")
    if ctrl is None:
        return
    
    # 후진 비동기 액션
    print("[Test] 비동기 후진")
    ctrl.execute_async(("drive_backward", (0.3, 0.2)))
    time.sleep(0.1)
    assert ctrl.is_action_running(), "후진 액션이 실행 중이어야 함"
    
    while ctrl.is_action_running():
        time.sleep(0.1)
    print("[OK] 후진 완료")
    
    # 원형 주행 비동기 액션
    print("[Test] 비동기 원형 주행")
    ctrl.execute_async(("drive_circle", (0.5, 0.3, 1.0)))
    time.sleep(0.1)
    assert ctrl.is_action_running(), "원형 주행 액션이 실행 중이어야 함"
    
    while ctrl.is_action_running():
        time.sleep(0.1)
    print("[OK] 원형 주행 완료")
    
    # 회전 비동기 액션
    print("[Test] 비동기 회전")
    ctrl.execute_async(("rotate", (90.0, 1.0)))
    time.sleep(0.1)
    assert ctrl.is_action_running(), "회전 액션이 실행 중이어야 함"
    
    while ctrl.is_action_running():
        time.sleep(0.1)
    print("[OK] 회전 완료")
    
    # 회전 액션 (각속도 없이)
    print("[Test] 비동기 회전 (각속도 기본값)")
    ctrl.execute_async(("rotate", 45.0))
    time.sleep(0.1)
    assert ctrl.is_action_running(), "회전 액션이 실행 중이어야 함"
    
    while ctrl.is_action_running():
        time.sleep(0.1)
    print("[OK] 회전 완료")
    
    print("[PASS] Test 9: Various Async Actions")


def test_speed_conversion():
    """속도 변환 테스트"""
    print("\n=== Test 10: Speed Conversion ===")
    
    ctrl = TikiController()
    _register_controller(ctrl)
    
    # 속도 변환 테스트
    test_speeds = [0.0, 0.25, 0.5, 1.0, -0.5]
    
    for speed_mps in test_speeds:
        rpm = ctrl._convert_speed_to_rpm(speed_mps)
        assert -127 <= rpm <= 127, "RPM 값은 -127~127 범위여야 함"
        print("[OK] {} m/s -> {} RPM".format(speed_mps, rpm))
    
    # 차동 주행 변환 테스트
    print("[Test] 차동 주행 변환")
    left_rpm, right_rpm = ctrl._convert_angular_to_differential(0.5, 0.5)
    assert -127 <= left_rpm <= 127, "좌측 RPM 값은 -127~127 범위여야 함"
    assert -127 <= right_rpm <= 127, "우측 RPM 값은 -127~127 범위여야 함"
    print("[OK] v=0.5, w=0.5 -> left={}, right={}".format(left_rpm, right_rpm))
    
    print("[PASS] Test 10: Speed Conversion")


def main():
    """모든 테스트 실행"""
    print("=" * 60)
    print("TikiController 소프트웨어 테스트")
    print("=" * 60)
    
    try:
        test_initialization()
        # test_brake()
        # test_update_speed_angular()
        test_drive_forward_distance()
        # test_drive_backward_distance()
        # test_drive_circle_distance()
        # test_rotate_in_place()
        # test_async_action()
        # test_async_actions()
        # test_speed_conversion()
        
        print("\n" + "=" * 60)
        print("[SUCCESS] All tests passed!")
        print("=" * 60)
    except KeyboardInterrupt:
        print("\n[INTERRUPTED] KeyboardInterrupt received. Stopping controllers...")
    except AssertionError as e:
        print("\n[FAIL] Assertion failed: {}".format(e))
        import traceback
        traceback.print_exc()
    except Exception as e:
        print("\n[ERROR] Test failed: {}".format(e))
        import traceback
        traceback.print_exc()
    finally:
        _cleanup_controllers()


if __name__ == "__main__":
    main()

