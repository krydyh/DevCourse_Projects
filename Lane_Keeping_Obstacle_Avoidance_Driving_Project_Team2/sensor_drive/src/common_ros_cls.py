#!/usr/bin/env python3
# -*- coding: utf-8 -*-

####################################################################
# 프로그램명 : common_ros_cls.py
# 모 델 명 : ALL
# 작 성 자 : 자이트론
# 생 성 일 : 2021년 03월 25일
# 수 정 일 : 2021년 03월 30일
# 검 수 인 : 조 이현
# 본 프로그램은 상업 라이센스에 의해 제공되므로 무단 배포 및 상업적 이용을 금합니다.
####################################################################

import rospy, time
from sensor_msgs.msg import LaserScan
from xycar_msgs.msg import xycar_motor
from std_msgs.msg import Int32MultiArray

class ros_module:
    lid_data = []
    ult_data = []

    lid_increment = 0

    def __init__(self, node_name):
        rospy.init_node(node_name)
        self.subscriber_init()
        self.publisher_init()
        self.msg_init()

    def subscriber_init(self):
        rospy.Subscriber('/xycar_ultrasonic',             Int32MultiArray, self.ros_ult_callback)
        rospy.Subscriber('/scan',                         LaserScan,       self.ros_lid_callback)

    def publisher_init(self):
        self.motor_pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)

    def msg_init(self):
        self.motor_msg = xycar_motor()
        self.motor_msg.header.frame_id = "xymotor"

    def rospy_is_shutdown(self):
        return rospy.is_shutdown()
        
    def motor_msg_pub(self, angle, speed):
    	self.motor_msg.angle = angle
    	self.motor_msg.speed = speed
        # self.motor_msg.header.stamp = rospy.get_rostime()

    	self.motor_pub.publish(self.motor_msg)

    def ros_ult_callback(self, data):
        self.ult_data = data.data

    def ros_lid_callback(self, data):
        self.lid_data = data.ranges
        self.lid_increment = data.angle_increment

    def get_ultrasonic_data(self):
        return self.ult_data

    def get_lidar_data(self):
        return self.lid_data, self.lid_increment



