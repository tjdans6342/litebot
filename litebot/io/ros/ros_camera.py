#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
ROSCamera 클래스
ROS 환경에서 카메라 이미지를 받아오는 클래스
"""
import numpy as np
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from litebot.io.camera_interface import CameraInterface


class ROSCamera(CameraInterface):
    """
    ROS 환경에서 카메라 이미지를 받아오는 클래스
    """
    
    def __init__(self, topic_name='/usb_cam/image_raw/compressed'):
        """
        ROSCamera 초기화
        
        Args:
            topic_name: ROS 이미지 토픽 이름 (기본값: '/camera/image_raw')
        """
        self.topic_name = topic_name
        self.latest_frame = None
        self.bridge = CvBridge()
        
        # ROS 이미지 토픽 구독
        self.image_sub = rospy.Subscriber(
            self.topic_name, 
            Image, 
            self._image_callback
        )
    
    def _image_callback(self, msg):
        """
        ROS 이미지 토픽 콜백 함수
        
        Args:
            msg: sensor_msgs.msg.Image 메시지
        """
        try:
            # ROS 이미지 메시지를 OpenCV 이미지로 변환 (BGR 형식)
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
            self.latest_frame = cv_image
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {}".format(e))
            self.latest_frame = None
    
    def get_frame(self):
        """
        최신 프레임을 반환합니다.
        
        Returns:
            np.ndarray: BGR 형식의 이미지 (height, width, 3)
            또는 None (프레임이 없는 경우)
        """
        return self.latest_frame
