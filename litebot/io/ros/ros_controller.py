#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
ROSController 클래스
ROS 환경에서 로봇을 제어하는 클래스
"""
import rospy
from litebot.io.controller_interface import ControllerInterface


class ROSController(ControllerInterface):
    """
    ROS 환경에서 로봇을 제어하는 클래스
    """
    
    def __init__(self):
        """ROSController 초기화"""
        # TODO: ROS 제어 토픽 Publisher 설정
        # self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        pass
    
    def brake(self):
        """정지"""
        # TODO: 실제 ROS 제어 명령 전송
        rospy.loginfo("Brake command")
        # self.cmd_vel_pub.publish(Twist())  # 모든 속도 0으로 설정
    
    def set_steering(self, value):
        """
        조향 설정
        
        Args:
            value: 조향 값
        """
        # TODO: 실제 ROS 제어 명령 전송
        rospy.loginfo("Set steering: {}".format(value))
    
    def drive_forward_distance(self, distance, speed):
        """
        거리 기반 전진
        
        Args:
            distance: 전진할 거리 (m)
            speed: 속도 (m/s)
        """
        # TODO: 실제 ROS 제어 명령 전송 (거리 기반 제어)
        rospy.loginfo("Drive forward: {} m at {} m/s".format(distance, speed))
    
    def drive_backward_distance(self, distance, speed):
        """
        거리 기반 후진
        
        Args:
            distance: 후진할 거리 (m)
            speed: 속도 (m/s)
        """
        # TODO: 실제 ROS 제어 명령 전송 (거리 기반 제어)
        rospy.loginfo("Drive backward: {} m at {} m/s".format(distance, speed))
    
    def drive_circle_distance(self, distance, speed, angular_velocity):
        """
        거리 기반 원형 주행
        
        Args:
            distance: 주행할 거리 (m)
            speed: 선속도 (m/s)
            angular_velocity: 각속도 (라디안/초)
        """
        # TODO: 실제 ROS 제어 명령 전송 (거리 기반 제어)
        rospy.loginfo("Drive circle: {} m at {} m/s with {} rad/s".format(distance, speed, angular_velocity))
