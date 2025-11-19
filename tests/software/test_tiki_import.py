#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Tiki 라이브러리 import 확인 테스트
tiki.mini 모듈이 정상적으로 불러와지는지 확인
"""
from __future__ import print_function

import sys


def test_tiki_import():
    """tiki.mini 모듈 import 테스트"""
    print("=" * 60)
    print("Tiki 라이브러리 Import 테스트")
    print("=" * 60)
    
    try:
        print("\n[Test] tiki.mini 모듈 import 시도...")
        from tiki.mini import TikiMini
        print("[OK] tiki.mini.TikiMini import 성공!")
        
        # TikiMini 인스턴스 생성 테스트
        print("\n[Test] TikiMini 인스턴스 생성 시도...")
        tiki = TikiMini()
        print("[OK] TikiMini 인스턴스 생성 성공!")
        
        # 주요 메서드/속성 확인
        print("\n[Test] 주요 속성/메서드 확인...")
        attrs_to_check = [
            'MOTOR_LEFT',
            'MOTOR_RIGHT',
            'MOTOR_MODE_PWM',
            'MOTOR_MODE_PID',
            'set_motor_mode',
            'set_motor_power',
            'stop',
            'get_encoder',
            'get_battery_voltage',
            'get_current',
            'get_imu',
            'set_led',
            'log',
        ]
        
        for attr in attrs_to_check:
            if hasattr(tiki, attr):
                print("[OK] {} 존재 확인".format(attr))
            else:
                print("[WARN] {} 없음".format(attr))
        
        print("\n" + "=" * 60)
        print("[SUCCESS] Tiki 라이브러리가 정상적으로 불러와집니다!")
        print("=" * 60)
        return True
        
    except ImportError as e:
        print("\n[ERROR] Import 실패: {}".format(e))
        print("[INFO] tiki.mini 모듈을 찾을 수 없습니다.")
        print("[INFO] 다음을 확인하세요:")
        print("  1. tiki 라이브러리가 설치되어 있는지")
        print("  2. Python 경로가 올바른지")
        print("  3. Jetson Nano 환경에서 실행 중인지")
        print("\n" + "=" * 60)
        return False
        
    except Exception as e:
        print("\n[ERROR] 예상치 못한 오류: {}".format(e))
        import traceback
        traceback.print_exc()
        print("\n" + "=" * 60)
        return False


def test_tiki_controller_import():
    """TikiController에서 tiki 사용 가능 여부 확인"""
    print("\n" + "=" * 60)
    print("TikiController Import 테스트")
    print("=" * 60)
    
    try:
        from litebot.io.tiki.tiki_controller import TikiController, TIKI_AVAILABLE
        
        print("\n[Test] TikiController import 성공")
        print("[INFO] TIKI_AVAILABLE = {}".format(TIKI_AVAILABLE))
        
        if TIKI_AVAILABLE:
            print("[OK] TikiController에서 tiki.mini 사용 가능")
        else:
            print("[WARN] TikiController에서 tiki.mini 사용 불가 (stub 모드)")
        
        # TikiController 인스턴스 생성 테스트
        print("\n[Test] TikiController 인스턴스 생성...")
        ctrl = TikiController()
        print("[OK] TikiController 인스턴스 생성 성공")
        print("[INFO] 모터 모드: {}".format(ctrl.motor_mode))
        
        return True
        
    except Exception as e:
        print("\n[ERROR] TikiController import 실패: {}".format(e))
        import traceback
        traceback.print_exc()
        return False


def main():
    """모든 테스트 실행"""
    success = True
    
    # 1. tiki.mini 직접 import 테스트
    success &= test_tiki_import()
    
    # 2. TikiController를 통한 간접 테스트
    success &= test_tiki_controller_import()
    
    if success:
        print("\n" + "=" * 60)
        print("[SUCCESS] 모든 테스트 통과!")
        print("=" * 60)
        sys.exit(0)
    else:
        print("\n" + "=" * 60)
        print("[FAIL] 일부 테스트 실패")
        print("=" * 60)
        sys.exit(1)


if __name__ == "__main__":
    main()

