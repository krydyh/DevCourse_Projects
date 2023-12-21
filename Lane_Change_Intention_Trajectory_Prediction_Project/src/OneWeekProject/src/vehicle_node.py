#!/usr/bin/env python3
#-*- coding:utf-8 -*-
import pickle

import numpy as np
import random
import rospy
import math
import copy
import rospkg
import time
import os
from PIL import Image
import matplotlib.pyplot as plt
import glob
import scipy.io as sio
import tf

from geometry_msgs.msg import Twist, Point32, PolygonStamped, Polygon, Vector3, Pose, Quaternion, Point
from visualization_msgs.msg import MarkerArray, Marker

from std_msgs.msg import Float32, Float64, Header, ColorRGBA, UInt8, String, Float32MultiArray, Int32MultiArray

   
        
      
class Environments(object):
    def __init__(self):
        rospy.init_node('Environments')

        self.init_variable()
        self.set_subscriber()
        self.set_publisher()
        
        ## 50ms 주기 
        r = rospy.Rate(20)
        while not rospy.is_shutdown():
            
            self.loop()
            r.sleep()
            


       
    def init_variable(self):
        
        ######################################## 
        ########### Load Sample ################
        ########################################
        
        SamplePath = rospy.get_param("SamplePath")        
        SampleList = sorted(glob.glob(SamplePath+"/*.pickle"))
        SampleId = (int)(rospy.get_param("SampleId"))
        
        with open(SampleList[SampleId], 'rb') as f:
            Data = pickle.load(f)
      
        self.Map = {"Lane" : Data["Map"],
                    "AP" : Data["Connectivity"],
                    "AL" : Data["Neighbor"]}
        
        self.Track = Data["Track"]
        
        
        
        self.pause = False
        self.dt = 0.05
        self.time = (int)(1/self.dt)+1


    def loop(self):
        
        if self.pause:
            pass
        else:
            self.pub_track()
            self.pub_map()
            self.pub_predict()
            
            self.time+=1
        
        if self.time>=(len(self.Track[0])-1):
            rospy.signal_shutdown("End of the logging Time")
            

    def pub_predict(self):
        
        #######################################
        ############### To Do #################
        #######################################
        
        
        Predicts = MarkerArray()
        
        for i in range(len(self.Track)):

            line_strip = Marker()
            line_strip.type = Marker.POINTS
            line_strip.id = i
            line_strip.scale.x = 1
            line_strip.scale.y = 1
            line_strip.scale.z = 1
            
            line_strip.color = ColorRGBA(0.0,0.0,1.0,0.5)
            line_strip.header = Header(frame_id='base_link')

            #############
            ### To Do ###
            #############
            """
            아래쪽의 "temp" 값에 예시로 GT 값을 집어 넣어놨는데요,
            이 부분에 여러분들이 다양한 방법을 통해서 찾아낸 Prediction 값을 넣어주세요.
            예측 사이즈는 몇 초 간격으로 예측하냐에 따라 달라지겠지만,
            0.05초 간격으로 2초를 예측한다면, 각 차량마다 [40,2]의 크기가 되겠죠?     
            """
             
            # temp = self.Track[i][self.time+1:self.time+41,:2]
            prev_data = self.Track[i][self.time-20:self.time+1,:2]
            prev_point = prev_data[-1]
            delta = prev_data[-1] - prev_data[-2]

            future_point = [prev_point]
            for _ in range(40): 
                next_point = future_point[-1] + delta
                future_point.append(next_point)

            temp = np.array(future_point)

            for j in range(len(temp)):
                point = Point()
                point.x = temp[j,0]
                point.y = temp[j,1]
                point.z = 2
                
                line_strip.points.append(point)
                
            Predicts.markers.append(line_strip) 
        
        self.predict_plot.publish(Predicts)
                
        
    def pub_track(self):
       
        Objects = MarkerArray()
        Historys = MarkerArray()
        Texts = MarkerArray()
        
        
        for i in range(len(self.Track)):
            
            ## Visualizae Object Cube and ID         
            q = tf.transformations.quaternion_from_euler(0, 0, self.Track[i][self.time][2])
            
            marker = Marker()
            marker.header.frame_id = "base_link"
            marker.header.stamp = rospy.Time.now()
            marker.id = i
            marker.type = Marker.CUBE
            marker.pose.position.x = self.Track[i][self.time][0] 
            marker.pose.position.y = self.Track[i][self.time][1] 
            marker.pose.position.z = 0.5
            marker.pose.orientation.x = q[0]
            marker.pose.orientation.y = q[1]
            marker.pose.orientation.z = q[2]
            marker.pose.orientation.w = q[3] 
            marker.scale.x = 4.5
            marker.scale.y = 2.4
            marker.scale.z = 3  
            marker.color.a = 1.0
                
            Objects.markers.append(marker)

            text = Marker()
            text.header.frame_id = "base_link"
            text.ns = "text"
            text.id = i
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.color = ColorRGBA(1, 1, 1, 1)
            text.scale.z = 5
            text.text = str(i)
            text.pose.position = Point(self.Track[i][self.time][0], self.Track[i][self.time][1], 3)
            
            Texts.markers.append(text)
            
            
            ## Visualizae History Trajectory
            line_strip = Marker()
            line_strip.type = Marker.POINTS
            line_strip.id = i
            line_strip.scale.x = 1
            line_strip.scale.y = 1
            line_strip.scale.z = 1
            
            line_strip.color = ColorRGBA(1.0,0.0,0.0,0.5)
            line_strip.header = Header(frame_id='base_link')

            temp = self.Track[i][self.time-20:self.time+1,:2]
            
            for j in range(len(temp)):
                point = Point()
                point.x = temp[j,0]
                point.y = temp[j,1]
                point.z = 2
                
                line_strip.points.append(point)
                
            Historys.markers.append(line_strip) 
            

        self.sur_pose_plot.publish(Objects)
        self.text_plot.publish(Texts)
        self.history_plot.publish(Historys)
        
        
        

    def pub_map(self, is_delete = False):   
        Maps = MarkerArray()
        
        for i in range(len(self.Map["Lane"])):
            line_strip = Marker()
            line_strip.type = Marker.LINE_STRIP
            line_strip.id = i
            line_strip.scale.x = 2
            line_strip.scale.y = 0.1
            line_strip.scale.z = 0.1
            
            line_strip.color = ColorRGBA(1.0,1.0,1.0,0.5)
            line_strip.header = Header(frame_id='base_link')

            temp = self.Map["Lane"][i]
            for j in range(len(temp)):
                point = Point()
                point.x = temp[j,0]
                point.y = temp[j,1]
                point.z = 0
                
                line_strip.points.append(point)
                
            Maps.markers.append(line_strip) 
            
        self.map_plot.publish(Maps)
    
    def set_subscriber(self):
        rospy.Subscriber('/cmd_vel',Twist, self.callback_plot,queue_size=1)   
        
    def set_publisher(self):
        
        # Object Cube & ID
        self.sur_pose_plot = rospy.Publisher('/rviz/sur_obj_pose', MarkerArray, queue_size=1)
        self.text_plot = rospy.Publisher('/rviz/text', MarkerArray, queue_size=1)

        # Map Point
        self.map_plot = rospy.Publisher('/rviz/maps', MarkerArray, queue_size=1)
        
        # 과거 1s 경로(0.05s 간격)
        self.history_plot = rospy.Publisher('/rviz/historys', MarkerArray, queue_size=1)
        
        # 예측 경로 2s
        self.predict_plot = rospy.Publisher('/rviz/predicts', MarkerArray, queue_size=1)
        
                   
            
    def callback_plot(self, data):
            
        if data.linear.x>0 and data.angular.z>0: #u
            self.pause = True 
        else:
            self.pause = False
    
    
        

if __name__ == '__main__':

    try:
        f = Environments()

    except rospy.ROSInterruptException:
        rospy.logerr('Could not start node.')

