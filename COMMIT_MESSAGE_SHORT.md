feat: Tiki 모드 완전 지원 추가

Tiki 하드웨어를 위한 완전한 지원을 추가했습니다.

주요 변경사항:
- TikiController: 실제 Tiki API 연결, 인코더 기반 거리 제어, 차동 주행 구현
- TikiCamera: Jetson Nano GStreamer 카메라 지원
- TikiSensor: 배터리, IMU, 인코더 센서 읽기
- TikiLed: LED 제어 및 패턴 지원
- TikiOled: OLED 디스플레이 제어
- 예제 코드 3개 추가 (기본 실행, 센서 테스트, LED 테스트)
- Tiki 모드 상세 가이드 문서 추가 (docs/TIKI_MODE_GUIDE.md)
- README에 Tiki 모드 사용법 추가

mode="tiki"로 설정하면 ROS 없이도 Tiki 로봇 제어 가능.
Stub 모드 지원으로 라이브러리 없이도 테스트 가능.

