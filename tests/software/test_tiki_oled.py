#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
TikiOled 소프트웨어 테스트
- 초기화 테스트
- 단일 텍스트 출력 테스트
- 여러 줄 텍스트 출력 테스트
- 화면 초기화 테스트
"""
from __future__ import print_function

from litebot.io.tiki.tiki_oled import TikiOled


def test_initialization():
    """초기화 테스트"""
    print("\n=== Test 1: Initialization ===")
    
    oled = TikiOled()
    
    # 메서드 존재 확인
    assert hasattr(oled, 'log'), "log 메서드가 있어야 함"
    assert hasattr(oled, 'log_clear'), "log_clear 메서드가 있어야 함"
    assert hasattr(oled, 'log_multiline'), "log_multiline 메서드가 있어야 함"
    
    print("[OK] 초기화 성공")
    print("[OK] 메서드 존재 확인: log, log_clear, log_multiline")
    
    print("[PASS] Test 1: Initialization")


def test_log():
    """단일 텍스트 출력 테스트"""
    print("\n=== Test 2: log ===")
    
    oled = TikiOled()
    
    # 다양한 텍스트 출력
    test_texts = [
        "Hello, Tiki!",
        "Test 123",
        "한글 테스트",
        "Special chars: !@#$%^&*()",
        "Long text: " + "A" * 50,
        "Numbers: 1234567890",
        "",
    ]
    
    for text in test_texts:
        oled.log(text)
        print("[OK] 텍스트 출력: '{}'".format(text[:50]))  # 처음 50자만 출력
    
    print("[PASS] Test 2: log")


def test_log_clear():
    """화면 초기화 테스트"""
    print("\n=== Test 3: log_clear ===")
    
    oled = TikiOled()
    
    # 텍스트 출력
    oled.log("Before clear")
    print("[OK] 초기 텍스트 출력")
    
    # 화면 초기화
    oled.log_clear()
    print("[OK] 화면 초기화 완료")
    
    # 초기화 후 텍스트 출력
    oled.log("After clear")
    print("[OK] 초기화 후 텍스트 출력")
    
    print("[PASS] Test 3: log_clear")


def test_log_multiline_list():
    """여러 줄 텍스트 출력 테스트 (리스트)"""
    print("\n=== Test 4: log_multiline (list) ===")
    
    oled = TikiOled()
    
    # 리스트로 여러 줄 출력
    lines_list = [
        "Line 1",
        "Line 2",
        "Line 3",
        "Line 4",
    ]
    
    oled.log_multiline(lines_list)
    print("[OK] 리스트로 여러 줄 출력 완료: {}줄".format(len(lines_list)))
    
    # 빈 리스트 테스트
    oled.log_multiline([])
    print("[OK] 빈 리스트 처리 확인")
    
    # 단일 항목 리스트 테스트
    oled.log_multiline(["Single line"])
    print("[OK] 단일 항목 리스트 처리 확인")
    
    print("[PASS] Test 4: log_multiline (list)")


def test_log_multiline_string():
    """여러 줄 텍스트 출력 테스트 (문자열)"""
    print("\n=== Test 5: log_multiline (string) ===")
    
    oled = TikiOled()
    
    # 개행 문자로 구분된 문자열
    lines_string = "Line 1\nLine 2\nLine 3\nLine 4"
    
    oled.log_multiline(lines_string)
    print("[OK] 문자열로 여러 줄 출력 완료")
    
    # 단일 줄 문자열 테스트
    oled.log_multiline("Single line")
    print("[OK] 단일 줄 문자열 처리 확인")
    
    # 빈 문자열 테스트
    oled.log_multiline("")
    print("[OK] 빈 문자열 처리 확인")
    
    # 여러 개행 문자 테스트
    oled.log_multiline("Line 1\n\nLine 3\n\n\nLine 6")
    print("[OK] 여러 개행 문자 처리 확인")
    
    print("[PASS] Test 5: log_multiline (string)")


def test_log_sequence():
    """텍스트 출력 시퀀스 테스트"""
    print("\n=== Test 6: Log Sequence ===")
    
    oled = TikiOled()
    
    # 시퀀스 1: 초기화 -> 출력
    oled.log_clear()
    oled.log("Step 1")
    print("[OK] 시퀀스 1 완료")
    
    # 시퀀스 2: 출력 -> 초기화 -> 출력
    oled.log("Step 2a")
    oled.log_clear()
    oled.log("Step 2b")
    print("[OK] 시퀀스 2 완료")
    
    # 시퀀스 3: 여러 줄 -> 초기화 -> 단일 출력
    oled.log_multiline(["Step 3a", "Step 3b", "Step 3c"])
    oled.log_clear()
    oled.log("Step 3d")
    print("[OK] 시퀀스 3 완료")
    
    print("[PASS] Test 6: Log Sequence")


def test_data_types():
    """다양한 데이터 타입 출력 테스트"""
    print("\n=== Test 7: Data Types ===")
    
    oled = TikiOled()
    
    # 정수
    oled.log(123)
    print("[OK] 정수 출력: 123")
    
    # 실수
    oled.log(3.14159)
    print("[OK] 실수 출력: 3.14159")
    
    # 불린
    oled.log(True)
    oled.log(False)
    print("[OK] 불린 출력: True, False")
    
    # None (문자열로 변환되어야 함)
    oled.log(None)
    print("[OK] None 출력 확인")
    
    print("[PASS] Test 7: Data Types")


def test_long_text():
    """긴 텍스트 출력 테스트"""
    print("\n=== Test 8: Long Text ===")
    
    oled = TikiOled()
    
    # 매우 긴 텍스트
    long_text = "A" * 200
    oled.log(long_text)
    print("[OK] 긴 텍스트 출력 (200자)")
    
    # 여러 줄 긴 텍스트
    long_multiline = "\n".join(["Line " + str(i) + ": " + "B" * 50 for i in range(10)])
    oled.log_multiline(long_multiline)
    print("[OK] 여러 줄 긴 텍스트 출력 (10줄)")
    
    print("[PASS] Test 8: Long Text")


def test_special_characters():
    """특수 문자 출력 테스트"""
    print("\n=== Test 9: Special Characters ===")
    
    oled = TikiOled()
    
    # 특수 문자들
    special_chars = [
        "!@#$%^&*()",
        "[]{}|\\:;\"'<>?,./",
        "한글 테스트",
        "日本語テスト",
        "中文测试",
        "Русский тест",
    ]
    
    for chars in special_chars:
        oled.log(chars)
        print("[OK] 특수 문자 출력: '{}'".format(chars))
    
    print("[PASS] Test 9: Special Characters")


def main():
    """모든 테스트 실행"""
    print("=" * 60)
    print("TikiOled 소프트웨어 테스트")
    print("=" * 60)
    
    try:
        test_initialization()
        test_log()
        test_log_clear()
        test_log_multiline_list()
        test_log_multiline_string()
        test_log_sequence()
        test_data_types()
        test_long_text()
        test_special_characters()
        
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

