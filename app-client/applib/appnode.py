#!/usr/bin/env python
# -*- coding: utf-8 -*-
#  __  __ ____ _  __ ____ ___ __  __
#  \ \/ // __// |/ //  _// _ |\ \/ /
#   \  // _/ /    /_/ / / __ | \  / 
#   /_//___//_/|_//___//_/ |_| /_/  
# 
#   2023 Yeniay Teknofest Takımı
#   
#   Teknofest 2023 Karma Sürü Robotlar Yarışması 
#   için hazırlanmıştır. Crazyflie App projesi .
#  

#   appnode.py 

# - sudo apt-get install ros-noetic-cv-bridge
# - sudo apt-get install ros-noetic-vision-opencv

# - CMAKE feed needed
# - Image format 'bgr8'

import rospy
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Vector3
from std_msgs.msg import String
from sensor_msgs.msg import Image
from custom_msg.msg import pose
from custom_msg.msg import obstacle
from custom_msg.msg import general_parameters

from cv_bridge import CvBridge, CvBridgeError



class AppNode:
    
    def __init__(self):
        self.reset()

    def node_start(self):
        rospy.init_node('app_node',anonymous=True)
        
    def reset(self):
        self.image = None
        self.obs1 = False
        self.obs2 = False
        self.obstacle_list = []
        self.obs_name_list = []
   
    def sub_image(self,img_adress):
        self.image_top = rospy.Subscriber(img_adress,Image,self.read_img)

    def read_img(self,data):
        bridge = CvBridge()
        self.image = bridge.imgmsg_to_cv2(data,"bgr8")
        print('image_added')
        self.image_top.unregister()
               
    def sub_obs(self,adress_list):
        rospy.Subscriber('obstacle/'+adress_list[0],Vector3,self.read_obs1)
        rospy.Subscriber('obstacle/'+adress_list[1],Vector3,self.read_obs2)

    def read_obs1(self,data):
        if self.obs1 == False:
            self.obs1=True
            self.obstacle_list.append([data.x,data.y,data.z])

    def read_obs2(self,data):
        if self.obs2 == False:
            self.obs2=True
            self.obstacle_list.append([data.x,data.y,data.z])

#TEST

# if __name__ == '__main__':
#     try:
#         nd = AppNode()
#         nd.node_start()
#         nd.sub_obs(['obstacle_F1'])
#         while(1):
#             print(nd.obstacle_list)
#     except rospy.ROSInitException:
#         pass
