#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
    TikiOled 클래스
    Tiki 환경에서 OLED 디스플레이를 제어하는 클래스
"""
try:
    from tiki.mini import TikiMini
    TIKI_AVAILABLE = True
except ImportError:
    TIKI_AVAILABLE = False
    print("[TikiOled] Warning: tiki.mini not available. Using stub implementation.")


class TikiOled:
    """
        Tiki 환경에서 OLED 디스플레이를 제어하는 클래스
    """
    
    def __init__(self):
        """
            TikiOled 초기화
        """
        if not TIKI_AVAILABLE:
            print("[TikiOled] Running in stub mode (tiki.mini not available)")
            self.tiki = None
        else:
            self.tiki = TikiMini()
    
    def log(self, text):
        """
            OLED 화면에 텍스트 출력
            
            Args:
                text: 출력할 문자열
        """
        if self.tiki is None:
            print("[TikiOled] log: {}".format(text))
            return
        
        self.tiki.log(str(text))
    
    def log_clear(self):
        """
            OLED 화면 초기화
        """
        if self.tiki is None:
            print("[TikiOled] log_clear (stub)")
            return
        
        self.tiki.log_clear()
    
    def log_multiline(self, lines):
        """
            여러 줄 텍스트 출력 (각 줄을 개행 문자로 구분)
            
            Args:
                lines: 문자열 리스트 또는 개행 문자로 구분된 문자열
        """
        if isinstance(lines, str):
            lines = lines.split('\n')
        
        self.log_clear()
        for line in lines:
            self.log(line)

