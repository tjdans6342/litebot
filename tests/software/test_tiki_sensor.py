#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
TikiSensor 소프트웨어 테스트
- 초기화 테스트
- 배터리 전압/전류 읽기 테스트
- IMU 센서 읽기 테스트
- 인코더 읽기 테스트
"""
from __future__ import print_function

from litebot.io.tiki.tiki_sensor import TikiSensor


def test_initialization():
    """초기화 테스트"""
    print("\n=== Test 1: Initialization ===")
    
    sensor = TikiSensor()
    
    # 메서드 존재 확인
    assert hasattr(sensor, 'get_battery_voltage'), "get_battery_voltage 메서드가 있어야 함"
    assert hasattr(sensor, 'get_battery_current'), "get_battery_current 메서드가 있어야 함"
    assert hasattr(sensor, 'get_imu'), "get_imu 메서드가 있어야 함"
    assert hasattr(sensor, 'get_encoder'), "get_encoder 메서드가 있어야 함"
    assert hasattr(sensor, 'get_encoders'), "get_encoders 메서드가 있어야 함"
    
    print("[OK] 초기화 성공")
    print("[OK] 메서드 존재 확인 완료")
    
    print("[PASS] Test 1: Initialization")


def test_battery_voltage():
    """배터리 전압 읽기 테스트"""
    print("\n=== Test 2: get_battery_voltage ===")
    
    sensor = TikiSensor()
    
    # 전압 읽기 (여러 번)
    voltages = []
    for i in range(5):
        voltage = sensor.get_battery_voltage()
        voltages.append(voltage)
        
        # 전압은 float 타입이어야 함
        assert isinstance(voltage, (int, float)), "전압은 숫자 타입이어야 함"
        assert voltage >= 0.0, "전압은 0 이상이어야 함"
        
        print("[OK] 전압 읽기 {}회: {:.2f}V".format(i+1, voltage))
    
    # 평균 계산
    avg_voltage = sum(voltages) / len(voltages)
    print("[OK] 평균 전압: {:.2f}V".format(avg_voltage))
    
    print("[PASS] Test 2: get_battery_voltage")


def test_battery_current():
    """배터리 전류 읽기 테스트"""
    print("\n=== Test 3: get_battery_current ===")
    
    sensor = TikiSensor()
    
    # 전류 읽기 (여러 번)
    currents = []
    for i in range(5):
        current = sensor.get_battery_current()
        currents.append(current)
        
        # 전류는 float 타입이어야 함
        assert isinstance(current, (int, float)), "전류는 숫자 타입이어야 함"
        
        print("[OK] 전류 읽기 {}회: {:.2f}A".format(i+1, current))
    
    # 평균 계산
    avg_current = sum(currents) / len(currents)
    print("[OK] 평균 전류: {:.2f}A".format(avg_current))
    
    print("[PASS] Test 3: get_battery_current")


def test_imu():
    """IMU 센서 읽기 테스트"""
    print("\n=== Test 4: get_imu ===")
    
    sensor = TikiSensor()
    
    # IMU 읽기 (여러 번)
    for i in range(5):
        imu = sensor.get_imu()
        
        # IMU는 튜플이어야 함
        assert isinstance(imu, tuple), "IMU는 튜플이어야 함"
        assert len(imu) == 3, "IMU는 3개의 값을 가져야 함 (ax, ay, az)"
        
        ax, ay, az = imu
        
        # 각 값은 float 타입이어야 함
        assert isinstance(ax, (int, float)), "ax는 숫자 타입이어야 함"
        assert isinstance(ay, (int, float)), "ay는 숫자 타입이어야 함"
        assert isinstance(az, (int, float)), "az는 숫자 타입이어야 함"
        
        print("[OK] IMU 읽기 {}회: ax={:.2f}, ay={:.2f}, az={:.2f}".format(
            i+1, ax, ay, az))
    
    print("[PASS] Test 4: get_imu")


def test_encoder_string():
    """인코더 읽기 테스트 (문자열)"""
    print("\n=== Test 5: get_encoder (string) ===")
    
    sensor = TikiSensor()
    
    # 좌측 인코더 읽기
    left_encoder = sensor.get_encoder('LEFT')
    assert isinstance(left_encoder, int), "인코더 값은 정수여야 함"
    print("[OK] 좌측 인코더 읽기: {}".format(left_encoder))
    
    # 우측 인코더 읽기
    right_encoder = sensor.get_encoder('RIGHT')
    assert isinstance(right_encoder, int), "인코더 값은 정수여야 함"
    print("[OK] 우측 인코더 읽기: {}".format(right_encoder))
    
    # 소문자 테스트
    left_lower = sensor.get_encoder('left')
    right_lower = sensor.get_encoder('right')
    assert isinstance(left_lower, int), "소문자도 동작해야 함"
    assert isinstance(right_lower, int), "소문자도 동작해야 함"
    print("[OK] 소문자 문자열 처리 확인")
    
    print("[PASS] Test 5: get_encoder (string)")


def test_encoder_constant():
    """인코더 읽기 테스트 (상수)"""
    print("\n=== Test 6: get_encoder (constant) ===")
    
    sensor = TikiSensor()
    
    # tiki가 사용 가능한 경우에만 테스트
    if sensor.tiki is not None:
        # 상수로 인코더 읽기
        left_encoder = sensor.get_encoder(sensor.tiki.MOTOR_LEFT)
        assert isinstance(left_encoder, int), "인코더 값은 정수여야 함"
        print("[OK] 좌측 인코더 읽기 (상수): {}".format(left_encoder))
        
        right_encoder = sensor.get_encoder(sensor.tiki.MOTOR_RIGHT)
        assert isinstance(right_encoder, int), "인코더 값은 정수여야 함"
        print("[OK] 우측 인코더 읽기 (상수): {}".format(right_encoder))
    else:
        print("[SKIP] tiki.mini가 사용 불가능하여 상수 테스트 건너뜀")
    
    print("[PASS] Test 6: get_encoder (constant)")


def test_get_encoders():
    """양쪽 인코더 읽기 테스트"""
    print("\n=== Test 7: get_encoders ===")
    
    sensor = TikiSensor()
    
    # 양쪽 인코더 읽기 (여러 번)
    for i in range(5):
        encoders = sensor.get_encoders()
        
        # encoders는 튜플이어야 함
        assert isinstance(encoders, tuple), "encoders는 튜플이어야 함"
        assert len(encoders) == 2, "encoders는 2개의 값을 가져야 함 (left, right)"
        
        left, right = encoders
        
        # 각 값은 정수여야 함
        assert isinstance(left, int), "좌측 인코더는 정수여야 함"
        assert isinstance(right, int), "우측 인코더는 정수여야 함"
        
        print("[OK] 인코더 읽기 {}회: left={}, right={}".format(i+1, left, right))
    
    print("[PASS] Test 7: get_encoders")


def test_continuous_reading():
    """연속 읽기 테스트"""
    print("\n=== Test 8: Continuous Reading ===")
    
    sensor = TikiSensor()
    
    # 모든 센서를 연속으로 읽기
    print("[Test] 모든 센서 연속 읽기 (5회)")
    for i in range(5):
        voltage = sensor.get_battery_voltage()
        current = sensor.get_battery_current()
        imu = sensor.get_imu()
        left_enc, right_enc = sensor.get_encoders()
        
        print("[OK] {}회: V={:.2f}V, I={:.2f}A, IMU=({:.2f}, {:.2f}, {:.2f}), "
              "Encoders=({}, {})".format(
                  i+1, voltage, current, imu[0], imu[1], imu[2], left_enc, right_enc))
    
    print("[PASS] Test 8: Continuous Reading")


def test_sensor_consistency():
    """센서 일관성 테스트"""
    print("\n=== Test 9: Sensor Consistency ===")
    
    sensor = TikiSensor()
    
    # 같은 센서를 여러 번 읽어서 타입 일관성 확인
    voltages = [sensor.get_battery_voltage() for _ in range(10)]
    currents = [sensor.get_battery_current() for _ in range(10)]
    imus = [sensor.get_imu() for _ in range(10)]
    encoders = [sensor.get_encoders() for _ in range(10)]
    
    # 모든 전압이 같은 타입인지 확인
    assert all(isinstance(v, (int, float)) for v in voltages), "모든 전압이 숫자 타입이어야 함"
    print("[OK] 전압 타입 일관성 확인 (10회)")
    
    # 모든 전류가 같은 타입인지 확인
    assert all(isinstance(c, (int, float)) for c in currents), "모든 전류가 숫자 타입이어야 함"
    print("[OK] 전류 타입 일관성 확인 (10회)")
    
    # 모든 IMU가 같은 형태인지 확인
    assert all(isinstance(imu, tuple) and len(imu) == 3 for imu in imus), "모든 IMU가 3개 튜플이어야 함"
    print("[OK] IMU 형태 일관성 확인 (10회)")
    
    # 모든 인코더가 같은 형태인지 확인
    assert all(isinstance(enc, tuple) and len(enc) == 2 for enc in encoders), "모든 인코더가 2개 튜플이어야 함"
    print("[OK] 인코더 형태 일관성 확인 (10회)")
    
    print("[PASS] Test 9: Sensor Consistency")


def main():
    """모든 테스트 실행"""
    print("=" * 60)
    print("TikiSensor 소프트웨어 테스트")
    print("=" * 60)
    
    try:
        test_initialization()
        test_battery_voltage()
        test_battery_current()
        test_imu()
        test_encoder_string()
        test_encoder_constant()
        test_get_encoders()
        test_continuous_reading()
        test_sensor_consistency()
        
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

