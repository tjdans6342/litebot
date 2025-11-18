feat: Tiki 모드 완전 지원 및 통합

Tiki 하드웨어를 위한 완전한 지원을 추가했습니다. mode="tiki"로 설정하면
ROS 없이도 Tiki 로봇을 제어할 수 있습니다.

주요 변경사항:

## 새로운 모듈 추가

- **TikiController**: 실제 Tiki API 연결, 인코더 기반 거리 제어, 차동 주행 모델 구현
  - PWM/PID 모터 모드 지원
  - 비동기 액션 실행 지원
  - Stub 모드 지원 (라이브러리 없을 때)

- **TikiCamera**: Jetson Nano GStreamer 파이프라인을 사용한 카메라 구현
  - CSI 카메라 지원
  - 해상도/프레임레이트 설정 가능

- **TikiSensor**: 센서 데이터 읽기 클래스
  - 배터리 전압/전류
  - IMU 3축 가속도
  - 인코더 값 (좌/우)

- **TikiLed**: LED 제어 클래스
  - 상단/좌측/우측 LED 개별 제어
  - X, O, # 패턴 지원
  - 편의 함수 제공

- **TikiOled**: OLED 디스플레이 제어 클래스
  - 텍스트 출력
  - 여러 줄 출력 지원

## 예제 코드

- examples/tiki/1_run_simple.py: 기본 실행 예제
- examples/tiki/2_test_sensors.py: 센서 테스트
- examples/tiki/3_test_led.py: LED 패턴 테스트

## 문서화

- docs/TIKI_MODE_GUIDE.md: Tiki 모드 상세 사용 가이드
  - 캘리브레이션 방법
  - 설정 가이드
  - 문제 해결
  - 성능 최적화

- tests/hardware/README_TIKI.md: ROS → Tiki 테스트 변환 가이드

- README.md: Tiki 모드 사용법 및 가이드 링크 추가

## 기타

- requirements.txt: 의존성 목록 추가
- litebot/io/tiki/__init__.py: 모듈 export 정리

## 호환성

- 기존 ROS 모드와 완전 호환
- ControllerInterface, CameraInterface 구현으로 인터페이스 호환성 보장
- Stub 모드로 Tiki 라이브러리 없이도 테스트 가능

## 주의사항

실제 하드웨어 사용 시 다음 캘리브레이션 값 조정 필요:
- encoder_ticks_per_meter
- wheelbase
- max_speed_mps

자세한 내용은 docs/TIKI_MODE_GUIDE.md 참조

