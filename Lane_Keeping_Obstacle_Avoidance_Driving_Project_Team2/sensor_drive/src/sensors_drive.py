#!/usr/bin/env python3
# -*- coding: utf-8 -*-

####################################################################
# 프로그램명 : sensors_drive.py
# 모 델 명 : B
# 작 성 자 : 자이트론
# 생 성 일 : 2021년 03월 25일
# 수 정 일 : 2021년 03월 30일
# 검 수 인 : 조 이현
# 본 프로그램은 상업 라이센스에 의해 제공되므로 무단 배포 및 상업적 이용을 금합니다.
####################################################################

import time
from common_ros_cls import ros_module
from judge import judge
from safety_box import safety

time.sleep(3)

r = ros_module("sensor_drive")
j = judge()
s = safety()

while not r.rospy_is_shutdown():
    if not s.get_wait():
        ultra_data = r.get_ultrasonic_data()
        lidar_data, lidar_increment = r.get_lidar_data()
        j.set_yolo_detect(False)
        obstacle = j.obstacle_chk(ultra_data, lidar_data, lidar_increment)
        if obstacle == -1:
            continue
        remind_time = j.mode_and_action_checker(obstacle)
        s.set_safety_wait_time(remind_time)
        s.time_init()

    s.set_angle_and_speed(j.get_angle(), j.get_speed())
    angle, speed = s.safety_drive()
    r.motor_msg_pub(angle, speed)

