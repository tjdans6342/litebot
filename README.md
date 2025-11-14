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
│   ├── recording/               # 주행 데이터, 분석 기록용
│   │   ├── video_recorder.py
│   │   ├── video_to_images.py
│   │   ├── lane_analysis_recorder.py
│   │   ├── README_LANE_ANALYSIS.md
│   │   └── README_VIDEO_TO_IMAGES.md
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
│   ├── run_ros.py
│   └── run_tiki.py
│
├── tests/                       
│   └── ...
│
├── README.md
├── setup.py
└── requirements.txt
```
