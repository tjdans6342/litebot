#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
TikiLed 소프트웨어 테스트
- 초기화 테스트
- 개별 LED 제어 테스트
- 전체 LED 제어 테스트
- 패턴 설정 테스트
"""
from __future__ import print_function

from litebot.io.tiki.tiki_led import TikiLed


def test_initialization():
    """초기화 테스트"""
    print("\n=== Test 1: Initialization ===")
    
    led = TikiLed()
    
    # 상수 확인
    assert TikiLed.DIRECTION_TOP == 0, "DIRECTION_TOP은 0이어야 함"
    assert TikiLed.DIRECTION_LEFT == 1, "DIRECTION_LEFT는 1이어야 함"
    assert TikiLed.DIRECTION_RIGHT == 2, "DIRECTION_RIGHT는 2이어야 함"
    
    print("[OK] 초기화 성공")
    print("[OK] 상수 확인: TOP={}, LEFT={}, RIGHT={}".format(
        TikiLed.DIRECTION_TOP, TikiLed.DIRECTION_LEFT, TikiLed.DIRECTION_RIGHT))
    
    print("[PASS] Test 1: Initialization")


def test_set_led():
    """개별 LED 제어 테스트"""
    print("\n=== Test 2: set_led ===")
    
    led = TikiLed()
    
    # 상단 LED 테스트
    print("[Test] 상단 LED 개별 제어")
    for i in range(16):
        led.set_led(TikiLed.DIRECTION_TOP, i, 255, 0, 0)  # 빨간색
    print("[OK] 상단 LED 16개 설정 완료")
    
    # 좌측 LED 테스트
    print("[Test] 좌측 LED 개별 제어")
    for i in range(8):
        led.set_led(TikiLed.DIRECTION_LEFT, i, 0, 255, 0)  # 초록색
    print("[OK] 좌측 LED 8개 설정 완료")
    
    # 우측 LED 테스트
    print("[Test] 우측 LED 개별 제어")
    for i in range(8):
        led.set_led(TikiLed.DIRECTION_RIGHT, i, 0, 0, 255)  # 파란색
    print("[OK] 우측 LED 8개 설정 완료")
    
    # 다양한 색상 테스트
    print("[Test] 다양한 색상 테스트")
    colors = [
        (255, 0, 0),    # 빨간색
        (0, 255, 0),    # 초록색
        (0, 0, 255),    # 파란색
        (255, 255, 0),  # 노란색
        (255, 0, 255),  # 마젠타
        (0, 255, 255),  # 시안
        (255, 255, 255), # 흰색
        (128, 128, 128), # 회색
    ]
    
    for r, g, b in colors:
        led.set_led(TikiLed.DIRECTION_TOP, 0, r, g, b)
        print("[OK] 색상 설정: RGB({}, {}, {})".format(r, g, b))
    
    print("[PASS] Test 2: set_led")


def test_clear_all():
    """모든 LED 끄기 테스트"""
    print("\n=== Test 3: clear_all ===")
    
    led = TikiLed()
    
    # LED 켜기
    led.set_all_top(255, 255, 255)
    led.set_all_left(255, 255, 255)
    led.set_all_right(255, 255, 255)
    print("[OK] 모든 LED 켜기 완료")
    
    # 모든 LED 끄기
    led.clear_all()
    print("[OK] 모든 LED 끄기 완료")
    
    print("[PASS] Test 3: clear_all")


def test_set_all_top():
    """상단 전체 LED 제어 테스트"""
    print("\n=== Test 4: set_all_top ===")
    
    led = TikiLed()
    
    # 다양한 색상으로 상단 LED 전체 설정
    colors = [
        (255, 0, 0),    # 빨간색
        (0, 255, 0),    # 초록색
        (0, 0, 255),    # 파란색
        (255, 255, 255), # 흰색
    ]
    
    for r, g, b in colors:
        led.set_all_top(r, g, b)
        print("[OK] 상단 LED 전체 설정: RGB({}, {}, {})".format(r, g, b))
    
    print("[PASS] Test 4: set_all_top")


def test_set_all_left():
    """좌측 전체 LED 제어 테스트"""
    print("\n=== Test 5: set_all_left ===")
    
    led = TikiLed()
    
    # 다양한 색상으로 좌측 LED 전체 설정
    colors = [
        (255, 0, 0),    # 빨간색
        (0, 255, 0),    # 초록색
        (0, 0, 255),    # 파란색
    ]
    
    for r, g, b in colors:
        led.set_all_left(r, g, b)
        print("[OK] 좌측 LED 전체 설정: RGB({}, {}, {})".format(r, g, b))
    
    print("[PASS] Test 5: set_all_left")


def test_set_all_right():
    """우측 전체 LED 제어 테스트"""
    print("\n=== Test 6: set_all_right ===")
    
    led = TikiLed()
    
    # 다양한 색상으로 우측 LED 전체 설정
    colors = [
        (255, 0, 0),    # 빨간색
        (0, 255, 0),    # 초록색
        (0, 0, 255),    # 파란색
    ]
    
    for r, g, b in colors:
        led.set_all_right(r, g, b)
        print("[OK] 우측 LED 전체 설정: RGB({}, {}, {})".format(r, g, b))
    
    print("[PASS] Test 6: set_all_right")


def test_set_pattern():
    """패턴 설정 테스트"""
    print("\n=== Test 7: set_pattern ===")
    
    led = TikiLed()
    
    # X 패턴
    print("[Test] X 패턴 설정")
    led.set_pattern('X', (255, 0, 0))
    print("[OK] X 패턴 설정 완료")
    
    # O 패턴
    print("[Test] O 패턴 설정")
    led.set_pattern('O', (0, 255, 0))
    print("[OK] O 패턴 설정 완료")
    
    # # 패턴
    print("[Test] # 패턴 설정")
    led.set_pattern('#', (0, 0, 255))
    print("[OK] # 패턴 설정 완료")
    
    # 커스텀 패턴 (리스트)
    print("[Test] 커스텀 패턴 설정")
    custom_pattern = [0, 1, 2, 3, 12, 13, 14, 15]
    led.set_pattern(custom_pattern, (255, 255, 0))
    print("[OK] 커스텀 패턴 설정 완료")
    
    # 알 수 없는 패턴 처리
    print("[Test] 알 수 없는 패턴 처리")
    led.set_pattern('UNKNOWN', (255, 255, 255))
    print("[OK] 알 수 없는 패턴 처리 확인 (에러 없이 동작)")
    
    print("[PASS] Test 7: set_pattern")


def test_pattern_combinations():
    """패턴 조합 테스트"""
    print("\n=== Test 8: Pattern Combinations ===")
    
    led = TikiLed()
    
    # 다양한 색상으로 패턴 테스트
    patterns = ['X', 'O', '#']
    colors = [
        (255, 0, 0),    # 빨간색
        (0, 255, 0),    # 초록색
        (0, 0, 255),    # 파란색
        (255, 255, 0),  # 노란색
    ]
    
    for pattern in patterns:
        for r, g, b in colors:
            led.set_pattern(pattern, (r, g, b))
            print("[OK] 패턴 '{}' 색상 RGB({}, {}, {}) 설정".format(pattern, r, g, b))
    
    print("[PASS] Test 8: Pattern Combinations")


def test_direction_independence():
    """방향별 독립성 테스트"""
    print("\n=== Test 9: Direction Independence ===")
    
    led = TikiLed()
    
    # 각 방향을 다른 색상으로 설정
    led.set_all_top(255, 0, 0)      # 상단: 빨간색
    led.set_all_left(0, 255, 0)     # 좌측: 초록색
    led.set_all_right(0, 0, 255)     # 우측: 파란색
    
    print("[OK] 각 방향 독립적으로 색상 설정 완료")
    
    # 개별 LED 설정
    led.set_led(TikiLed.DIRECTION_TOP, 0, 255, 255, 255)  # 상단 첫 번째: 흰색
    led.set_led(TikiLed.DIRECTION_LEFT, 0, 128, 128, 128)  # 좌측 첫 번째: 회색
    led.set_led(TikiLed.DIRECTION_RIGHT, 0, 64, 64, 64)    # 우측 첫 번째: 어두운 회색
    
    print("[OK] 개별 LED 독립 설정 완료")
    
    print("[PASS] Test 9: Direction Independence")


def main():
    """모든 테스트 실행"""
    print("=" * 60)
    print("TikiLed 소프트웨어 테스트")
    print("=" * 60)
    
    try:
        test_initialization()
        test_set_led()
        test_clear_all()
        test_set_all_top()
        test_set_all_left()
        test_set_all_right()
        test_set_pattern()
        test_pattern_combinations()
        test_direction_independence()
        
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

