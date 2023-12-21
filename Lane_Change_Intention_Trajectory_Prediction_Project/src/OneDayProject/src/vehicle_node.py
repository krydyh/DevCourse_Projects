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
        self.load_map()
        
        r = rospy.Rate(20)
        while not rospy.is_shutdown():
            
            self.loop()
            r.sleep()
            

    def load_map(self):
        self.map_file = []
        map_path = rospy.get_param("map_path")
        matfiles = ["waypoints_0_rev.mat",
                    "waypoints_1_rev.mat",
                    "waypoints_2_rev.mat",
                    "waypoints_3_rev.mat"
                    ]


        
        for i, matfile in enumerate(matfiles):
            mat = sio.loadmat(map_path+matfile)
            
            easts = mat["east"][0]
            norths = mat["north"][0]
            stations = mat["station"][0]
            # [326107, 340838]    

            self.map_file.append(np.stack([easts, norths, stations],axis=-1))
        
        
        self.D_list = [ 0, -3.85535188 ,-7.52523438, -7.37178602]
        self.D_list = np.array(self.D_list) 

       
    def init_variable(self):
        self.pause = False
        self.time = 11
        self.br = tf.TransformBroadcaster()
      
        SamplePath = rospy.get_param("SamplePath")
        SampleList = sorted(glob.glob(SamplePath+"/*.pickle"))
        SampleId = (int)(rospy.get_param("SampleId"))
        
        ######################### Load Vehicle #######################    
        with open(SampleList[SampleId], 'rb') as f:
            self.vehicles = pickle.load(f)
        

    def loop(self):
        
        if self.pause:
            pass
        else:
            self.publish()
            self.pub_map()
            self.time+=1
        
        if self.time>=(len(self.vehicles[0])-1):
            rospy.signal_shutdown("End of the logging Time")
            
               
            
    
            
    def publish(self, is_delete = False):
        
        Objects = MarkerArray()
        Texts = MarkerArray()
        
        for i in range(len(self.vehicles)):
            
            ####################################################
            ###################### To Do #######################
            ####################################################
            
            """
            각 vehicle마다 self.vehicles 안에 위치한 정보를 통해서 과거 10개 step(0.5s)의 관측 값을 바탕으로,
            현재 차량의 차선 변경 의도를 예측하기.
            그리고, pred 변수에 LC 이면 "LC", LK이면 "LK" 라는 값을 대입하여 결과를 확인해 봅시다.
            """

            past_observation = self.vehicles[i][self.time - 10 : self.time]
            predict_intent = [observation[9] for observation in past_observation]
            fixed_intent = max(set(predict_intent), key=predict_intent.count)
            
            
            pred = "LC" if fixed_intent == 1 else "LK"
            
            gt = "LC" if self.vehicles[i][self.time][9] == 1 else "LK"
            
            
            ## marker_publish   
            q = tf.transformations.quaternion_from_euler(0, 0, self.vehicles[i][self.time][6])
            
            marker = Marker()
            marker.header.frame_id = "world"
            marker.header.stamp = rospy.Time.now()
            marker.id = i
            marker.type = Marker.CUBE
            
            marker.pose.position.x = self.vehicles[i][self.time][4] 
            marker.pose.position.y = self.vehicles[i][self.time][5] 
            marker.pose.position.z = 0.5
            
            marker.pose.orientation.x = q[0]
            marker.pose.orientation.y = q[1]
            marker.pose.orientation.z = q[2]
            marker.pose.orientation.w = q[3]
                
                
            marker.scale.x = self.vehicles[i][self.time][12] 
            marker.scale.y = self.vehicles[i][self.time][13] 
            marker.scale.z = 1    
            marker.color.a = 1.0
                
            
            Objects.markers.append(marker)

            
            text = Marker()
            text.header.frame_id = "world"
            text.ns = "text"
            text.id = i
            text.type = Marker.TEXT_VIEW_FACING
            
            text.action = Marker.ADD

            text.color = ColorRGBA(1, 1, 1, 1)
            text.scale.z = 5
            text.text = str(i)+" / True : " + gt+" / Pred : "  + pred
            text.pose.position = Point(self.vehicles[i][self.time][4], self.vehicles[i][self.time][5], 3)

            Texts.markers.append(text)
    
    
        self.sur_pose_plot.publish(Objects)
        self.text_plot.publish(Texts)
        
        
        self.br.sendTransform((self.vehicles[0][self.time][4], self.vehicles[0][self.time][5], 0),
                                tf.transformations.quaternion_from_euler(0, 0,self.vehicles[0][self.time][6]),
                                rospy.Time.now(),
                                "base_link",
                                "world")
        
    def pub_map(self, is_delete = False):   
        Maps = MarkerArray()
        
        for i in range(len(self.map_file)):
            line_strip = Marker()
            line_strip.type = Marker.LINE_STRIP
            line_strip.id = i
            line_strip.scale.x = 2
            line_strip.scale.y = 0.1
            line_strip.scale.z = 0.1
            
            line_strip.color = ColorRGBA(1.0,1.0,1.0,0.5)
            line_strip.header = Header(frame_id='world')

            temp = self.map_file[i]
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
        
        self.sur_pose_plot = rospy.Publisher('/rviz/sur_obj_pose', MarkerArray, queue_size=1)
        self.map_plot = rospy.Publisher('/rviz/maps', MarkerArray, queue_size=1)
        self.text_plot = rospy.Publisher('/rviz/text', MarkerArray, queue_size=1)
        
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

