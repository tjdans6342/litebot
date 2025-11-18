#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
    ROSController 클래스
    ROS 환경에서 로봇을 제어하는 클래스
"""
from __future__ import with_statement  # Python 2.7 호환성

import threading
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
        
        # 비동기 액션 실행을 위한 상태 변수
        self._action_thread = None
        self._action_running = False
        self._action_lock = threading.Lock()
    
    def _wait_for_connection(self):
        """
            Publisher가 최소 한 개의 구독자를 가질 때까지 잠시 대기
        """
        timeout = rospy.Time.now() + rospy.Duration(2.0)
        while not rospy.is_shutdown():
            if self.cmd_vel_pub.get_num_connections() > 0:
                break
            if rospy.Time.now() > timeout:
                rospy.logwarn("[ROSController] No subscribers on {} within timeout. Continuing anyway.".format(self.cmd_topic))
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
            ActionExecutor에서 실행 제어를 담당하므로 여기서는 단순 실행만 수행합니다.
        """
        rospy.loginfo("[ROSController] Brake command")
        self.current_speed = 0.0
        self._publish_once(0.0, 0.0)
    
    def update_speed_angular(self, linear_x, angular_z): # TODO: here to use PID Controller
        """
            선속도와 각속도를 동시에 갱신
            ActionExecutor에서 실행 제어를 담당하므로 여기서는 단순 실행만 수행합니다.
            
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
            rospy.logwarn("[ROSController] Invalid forward parameters distance={} speed={}".format(distance, speed))
            return
        
        rospy.loginfo("[ROSController] Drive forward: {} m at {} m/s".format(distance, speed))
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
            rospy.logwarn("[ROSController] Invalid backward parameters distance={} speed={}".format(distance, speed))
            return
        
        rospy.loginfo("[ROSController] Drive backward: {} m at {} m/s".format(distance, speed))
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
            rospy.logwarn("[ROSController] Invalid circle parameters distance={} speed={}".format(distance, speed))
            return
        
        rospy.loginfo(
            "[ROSController] Drive circle: {} m at {} m/s with {} rad/s".format(distance, speed, angular_velocity)
        )
        duration = distance / speed
        self.current_speed = speed
        self._publish_for_duration(speed, angular_velocity, duration)
    
    def rotate_in_place(self, degrees, ang_speed=1.0):
        """
            제자리 회전(선속도 0, 각속도 유지) - 시간 기반 수행
            
            Args:
                degrees (float): 회전할 각도 (deg). 양수: 좌회전, 음수: 우회전
                ang_speed (float): 각속도 크기 (rad/s, 양수)
        """
        if ang_speed <= 0.0:
            rospy.logwarn("[ROSController] rotate_in_place: ang_speed must be positive")
            return
        radians = abs(degrees) * 3.141592653589793 / 180.0
        duration = radians / ang_speed if ang_speed > 0.0 else 0.0
        angular = ang_speed if degrees >= 0.0 else -ang_speed
        rospy.loginfo("[ROSController] Rotate in place: {} deg at {} rad/s ({} s)".format(degrees, ang_speed, duration))
        self._publish_for_duration(0.0, angular, duration)
    
    # ========== 비동기 액션 실행 메서드 ==========

    def _run_action(self, action):
        """
            스레드에서 실행되는 액션 실행 메서드
        """
        try:
            cmd, value = action
            if cmd == "drive_forward":
                distance, speed = value
                self.drive_forward_distance(distance, speed)
            elif cmd == "drive_backward":
                distance, speed = value
                self.drive_backward_distance(distance, speed)
            elif cmd == "drive_circle":
                distance, speed, angular_velocity = value
                self.drive_circle_distance(distance, speed, angular_velocity)
            elif cmd == "rotate":
                # value는 (degrees, ang_speed) 형태
                if isinstance(value, tuple) and len(value) == 2:
                    degrees, ang_speed = value
                else:
                    # 하위 호환성: 단일 값이면 기본 각속도 사용
                    degrees = value
                    ang_speed = 1.0
                self.rotate_in_place(degrees, ang_speed)
            else:
                rospy.logwarn("[ROSController] Unknown async action: {}".format(cmd))
        except Exception as e:
            rospy.logerr("[ROSController] Error in async action: {}".format(e))
        finally:
            with self._action_lock:
                self._action_running = False
                self._action_thread = None

    def _cancel_current_action(self):
        """
            현재 실행 중인 액션을 취소
            brake()를 호출하지 않고 직접 publish하여 재귀 호출 방지
        """
        if self._action_thread is not None and self._action_thread.is_alive():
            # 스레드가 살아있으면 직접 정지 명령 publish (brake() 호출하지 않음)
            self.current_speed = 0.0
            self._publish_once(0.0, 0.0)
            # 스레드 종료를 기다리지 않고 플래그만 설정 (스레드는 자연 종료)
            self._action_running = False 

    def execute_async(self, action):
        """
            액션을 비동기로 실행 (즉시 반환)
            
            Args:
                action: (command, value) 형태의 튜플
            
            Note:
                기존 액션이 실행 중이면 새 액션은 무시됩니다 (큐 방식).
                액션 완료를 기다리려면 ActionExecutor나 TriggerManager에서 처리해야 합니다.
        """
        with self._action_lock:
            # 기존 액션이 실행 중이면 새 액션 무시 (큐 방식)
            if self._action_running and self._action_thread is not None and self._action_thread.is_alive():
                rospy.logwarn("[ROSController] Action already running, ignoring new action: {}".format(action[0]))
                return
            
            # 새 액션을 스레드로 실행
            self._action_running = True
            self._action_thread = threading.Thread(target=self._run_action, args=(action,))
            # daemon이 뭐지? 쓰레드가 메인 프로세스가 끝나면 같이 종료되게 설정하는 옵션임
            self._action_thread.daemon = True
            self._action_thread.start()
    
    def is_action_running(self):
        """
            현재 액션이 실행 중인지 확인
        """
        with self._action_lock:
            return self._action_running and self._action_thread is not None and self._action_thread.is_alive()
    
    def cancel_action(self):
        """
            현재 실행 중인 액션을 취소하고 정지
        """
        with self._action_lock:
            self._cancel_current_action()