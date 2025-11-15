```
litebot/
├── litebot/                       
│   ├── core/                    
│   │   ├── observer.py          
│   │   ├── trigger_manager.py   
│   │   ├── trigger_lane.py
│   │   ├── trigger_aruco.py
│   │   ├── trigger_pothole.py
│   │   └── trigger_qrcode.py
│   │
│   ├── action/                  
│   │   └── action_executor.py
│   │
│   ├── io/                      
│   │   ├── camera_interface.py
│   │   ├── controller_interface.py
│   │   ├── ros/                 
│   │   │   ├── ros_camera.py
│   │   │   └── ros_controller.py
│   │   └── tiki/                
│   │       ├── tiki_camera.py
│   │       └── tiki_controller.py
│   │
│   ├── processing/              
│   │   ├── image_processor.py
│   │   ├── image_yolo.py
│   │   └── fire_detector.py
│   │
│   ├── analysis/                # 주행 데이터, 분석 기록용
│   │   ├── analysis_manager.py
│   │   └── video_recorder.py
│   │
│   ├── configs/                 # 감지, 추론, 주행 설정 등
│   │   ├── lane_config.py
│   │   ├── aruco_rules.py
│   │   └── video_config.py
│   │
│   └── utils/                   
│       ├── aruco_utils.py
│       ├── check_utils.py
│       └── image_utils.py
│
├── examples/
│   ├── ros/
│   │   ├── run_ros.py                  # 기본 실행 (튜토리얼 1단계와 동일)
│   │   ├── 1_run_simple.py             # 튜토리얼 1: 최소 구성
│   │   ├── 2_run_with_video.py         # 튜토리얼 2: 주행 + 비디오 녹화
│   │   ├── 3_run_parallel_obs.py       # 튜토리얼 3: 병렬 관측 처리
│   │   └── 4_run_with_detection.py     # 튜토리얼 4: DetectionBridge 연계
│   └── tiki/
│       └── run_tiki.py
│
├── ai/
│   ├── detection_bridge.py
│   └── object_detector.py
│
├── models/                    # YOLO 등 추론 가중치(.pt) 보관
│   └── (예: best_rokaf.pt)
│
├── tests/                       
│   └── ...
│
├── README.md
├── setup.py
└── requirements.txt
```

## 객체 감지 파이프라인

- `ai/object_detector.py`  
  - 실행 시 `detecting/to_detect_images_YYYYMMDD_HHMM` 세션 디렉터리와 `detected_YYYYMMDD_HHMM.log`를 자동 생성합니다.  
  - LiteBot이 세션 폴더에 저장한 이미지를 주기적으로 스캔하고 YOLO 추론 결과를 로그(JSON Lines)로 남깁니다.  
  - 현재 세션 정보는 `detecting/current_session.json`에 기록되며, 다른 프로세스가 참조할 수 있습니다.

- `ai/detection_bridge.py`  
  - LiteBot 측에서 `current_session.json`을 읽어 최신 세션을 추적하고, 프레임을 HHMMSS.jpg 형태로 저장하거나 새로 추가된 감지 로그를 스트리밍할 수 있는 헬퍼 클래스입니다.

### 실행 예시

```bash
# 객체 감지 워커 실행 (가중치는 models/ 아래에 위치)
# --conf  : YOLO confidence threshold (예: 0.35)
# --sleep : 폴더 폴링 주기 (초 단위)
python ai/object_detector.py --model models/best_rokaf.pt --conf 0.35 --sleep 0.5
```

```python
# LiteBot 측에서 감지 세션에 프레임 저장 예시
from ai.detection_bridge import DetectionBridge

bridge = DetectionBridge()
bridge.save_frame(frame)
logs = bridge.read_new_logs()
```