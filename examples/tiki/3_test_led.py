#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Tiki LED 테스트 예제
LED 패턴을 표시합니다.
"""
import os
import sys
import time

# 프로젝트 루트를 Python path에 추가
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, PROJECT_ROOT)

from litebot.io.tiki import TikiLed  # noqa: E402


def main():
    """
        Tiki LED 테스트
    """
    print("[Tiki LED Test] Initializing...")
    led = TikiLed()
    
    print("[Tiki LED Test] Starting LED pattern test...")
    print("[Tiki LED Test] Press Ctrl+C to stop")
    
    try:
        # 1. 상단 LED 모두 빨간색
        print("Setting all top LEDs to RED...")
        led.set_all_top(50, 0, 0)
        time.sleep(2)
        
        # 2. 상단 LED 모두 초록색
        print("Setting all top LEDs to GREEN...")
        led.set_all_top(0, 50, 0)
        time.sleep(2)
        
        # 3. 상단 LED 모두 파란색
        print("Setting all top LEDs to BLUE...")
        led.set_all_top(0, 0, 50)
        time.sleep(2)
        
        # 4. 패턴 테스트
        patterns = ['X', 'O', '#']
        colors = [
            (50, 0, 0),    # 빨간색
            (0, 50, 0),    # 초록색
            (0, 0, 50),    # 파란색
        ]
        
        for pattern, color in zip(patterns, colors):
            print("Displaying pattern: {}".format(pattern))
            led.set_pattern(pattern, color)
            time.sleep(2)
        
        # 5. 좌우 LED 테스트
        print("Setting left LEDs to RED, right LEDs to BLUE...")
        led.set_all_left(50, 0, 0)
        led.set_all_right(0, 0, 50)
        time.sleep(2)
        
        # 6. 모두 끄기
        print("Turning off all LEDs...")
        led.clear_all()
        time.sleep(1)
        
        print("[Tiki LED Test] Test completed")
        
    except KeyboardInterrupt:
        print("\n[Tiki LED Test] Stopping...")
    finally:
        led.clear_all()
        print("[Tiki LED Test] Done")


if __name__ == "__main__":
    main()

