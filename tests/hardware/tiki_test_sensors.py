#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Tiki 센서 테스트
배터리, IMU, 인코더 값을 읽어서 표시합니다.
"""
import os
import sys
import time

# 프로젝트 루트를 Python path에 추가
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, PROJECT_ROOT)

from litebot.io.tiki import TikiSensor, TikiOled  # noqa: E402


def main():
    """
        Tiki 센서 테스트
    """
    print("[Tiki Sensor Test] Initializing...")
    sensor = TikiSensor()
    oled = TikiOled()
    
    print("[Tiki Sensor Test] Starting sensor reading loop...")
    print("[Tiki Sensor Test] Press Ctrl+C to stop")
    
    try:
        while True:
            # 배터리 정보
            voltage = sensor.get_battery_voltage()
            current = sensor.get_battery_current()
            
            # IMU 센서
            ax, ay, az = sensor.get_imu()
            
            # 인코더
            left_enc, right_enc = sensor.get_encoders()
            
            # 콘솔 출력
            print("\n" + "="*50)
            print("Battery: {:.2f}V, {:.2f}A".format(voltage, current))
            print("IMU: AX={:.1f}, AY={:.1f}, AZ={:.1f}".format(ax, ay, az))
            print("Encoder: Left={}, Right={}".format(left_enc, right_enc))
            
            # OLED 출력
            oled.log_clear()
            oled.log("V:{:.1f}V I:{:.1f}A".format(voltage, current))
            oled.log("AX:{:.0f} AY:{:.0f}".format(ax, ay))
            oled.log("L:{} R:{}".format(left_enc, right_enc))
            
            time.sleep(1.0)
            
    except KeyboardInterrupt:
        print("\n[Tiki Sensor Test] Stopping...")
    finally:
        oled.log_clear()
        oled.log("Test stopped")
        print("[Tiki Sensor Test] Done")


if __name__ == "__main__":
    main()

