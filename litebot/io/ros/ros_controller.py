#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
    ROSController 클래스
    ROS 환경에서 로봇을 제어하는 클래스
"""
import rospy
from geometry_msgs.msg import Twist
from litebot.io.controller_interface import ControllerInterface


class ROSController(ControllerInterface):
    """
        ROS 환경에서 로봇을 제어하는 클래스
    """
    
    def __init__(self, cmd_topic='/cmd_vel', rate_hz=20.0):
        """
            ROSController 초기화
            
            Args:
                cmd_topic (str): 속도 명령을 퍼블리시할 ROS 토픽
                rate_hz (float): 명령 퍼블리시 빈도
        """
        self.cmd_topic = cmd_topic
        self.rate_hz = rate_hz
        self.current_speed = 0.0
        self.cmd_vel_pub = rospy.Publisher(self.cmd_topic, Twist, queue_size=1)
        self._wait_for_connection()
    
    def _wait_for_connection(self):
        """
            Publisher가 최소 한 개의 구독자를 가질 때까지 잠시 대기
        """
        timeout = rospy.Time.now() + rospy.Duration(2.0)
        while not rospy.is_shutdown():
            if self.cmd_vel_pub.get_num_connections() > 0:
                break
            if rospy.Time.now() > timeout:
                rospy.logwarn("No subscribers on {} within timeout. Continuing anyway.".format(self.cmd_topic))
                break
            rospy.sleep(0.05)
    
    def _publish_once(self, linear_x, angular_z):
        """
            Twist 메시지를 한 번 퍼블리시
        """
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.cmd_vel_pub.publish(twist)
    
    def _publish_for_duration(self, linear_x, angular_z, duration):
        """
            지정된 시간(duration) 동안 속도 명령을 퍼블리시
            
            Args:
                linear_x (float): 선속도 (m/s)
                angular_z (float): 각속도 (rad/s)
                duration (float): 퍼블리시 시간 (초)
        """
        if duration <= 0.0:
            self._publish_once(linear_x, angular_z)
            self.brake()
            return
        
        rate = rospy.Rate(self.rate_hz)
        end_time = rospy.Time.now() + rospy.Duration(duration)
        while rospy.Time.now() < end_time and not rospy.is_shutdown():
            self._publish_once(linear_x, angular_z)
            rate.sleep()
        self.brake()
    
    def brake(self):
        """
            정지
        """
        rospy.loginfo("Brake command")
        self.current_speed = 0.0
        self._publish_once(0.0, 0.0)
    
    def set_steering(self, value):
        """
            조향 설정
            
            Args:
                value: 조향 값 (rad/s 형태로 가정)
        """
        rospy.loginfo("Set steering: {}".format(value))
        # 현재 선속도를 유지한 채 조향만 변경
        self._publish_once(self.current_speed, value)
    
    def update_speed_angular(self, linear_x, angular_z):
        """
            선속도와 각속도를 동시에 갱신
            
            Args:
                linear_x (float): 선속도
                angular_z (float): 각속도
        """
        self.current_speed = linear_x
        self._publish_once(linear_x, angular_z)
    
    def drive_forward_distance(self, distance, speed):
        """
            거리 기반 전진
            
            Args:
                distance: 전진할 거리 (m)
                speed: 속도 (m/s)
        """
        if distance <= 0.0 or speed <= 0.0:
            rospy.logwarn("Invalid forward parameters distance={} speed={}".format(distance, speed))
            return
        
        rospy.loginfo("Drive forward: {} m at {} m/s".format(distance, speed))
        duration = distance / speed
        self.current_speed = speed
        self._publish_for_duration(speed, 0.0, duration)
    
    def drive_backward_distance(self, distance, speed):
        """
            거리 기반 후진
            
            Args:
                distance: 후진할 거리 (m)
                speed: 속도 (m/s, 양수)
        """
        if distance <= 0.0 or speed <= 0.0:
            rospy.logwarn("Invalid backward parameters distance={} speed={}".format(distance, speed))
            return
        
        rospy.loginfo("Drive backward: {} m at {} m/s".format(distance, speed))
        duration = distance / speed
        # 후진은 음수 선속도
        self.current_speed = -speed
        self._publish_for_duration(-speed, 0.0, duration)
    
    def drive_circle_distance(self, distance, speed, angular_velocity):
        """
            거리 기반 원형 주행
            
            Args:
                distance: 주행할 거리 (m)
                speed: 선속도 (m/s)
                angular_velocity: 각속도 (라디안/초)
        """
        if distance <= 0.0 or speed <= 0.0:
            rospy.logwarn("Invalid circle parameters distance={} speed={}".format(distance, speed))
            return
        
        rospy.loginfo(
            "Drive circle: {} m at {} m/s with {} rad/s".format(distance, speed, angular_velocity)
        )
        duration = distance / speed
        self.current_speed = speed
        self._publish_for_duration(speed, angular_velocity, duration)
