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

#   pip install opencv-python
#   appcv.py 


import cv2 
import numpy as np

class AppCv:
    def __init__(self,image):
        self.image=image
        self.img = self.image.copy()

        self.mask = self.image_treshold(self.img)
        self.mask_outline = self.image_outline(self.mask,5)
        self.mask_outline2 = self.image_outline(self.mask_outline,2)
        self.points= self.image_contour(self.mask)
        self.outline_points=self.image_contour(self.mask_outline)
        self.outline_points2=self.image_contour(self.mask_outline2)
        self.image_show(self.img)
        
    
    def get_contour(self,praw):
        pcloud=[]
        for p in praw:
            pcloud.append([p[0][0],-p[0][1]])
        return pcloud

    def image_treshold(self,img):
        hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
        hsv = cv2.GaussianBlur(hsv,(21,21),0)
        
        
        #------------------------------------
        # """ Dark Object, White background """
        # lower = np.array([0,40,0])
        # upper = np.array([120,255,130])
        # mask = cv2.inRange(hsv,lower,upper)
        
        #------------------------------------
       
        # lower1 = np.array([0,70,50])
        # upper1 = np.array([10,255,255])

        # lower2 = np.array([0,40,0])
        # upper2 = np.array([80,255,255])

        # mask1 = cv2.inRange(hsv,lower1,upper1)
        # mask2 = cv2.inRange(hsv,lower2,upper2)
        # mask = mask1 | mask2
        # return mask
        
        #------------------------------------
        """ Red,Orange Object """
        lower = np.array([0,50,50])
        upper = np.array([60,255,255])

        mask = cv2.inRange(hsv,lower,upper)

        #------------------------------------
        #mask =cv2.bitwise_not(mask)
        mask = self.image_outline(mask,3)
        return mask

    def image_outline(self,img,f=1):
        mask_outline=img
        for i in range(f):
            mask_blur = cv2.GaussianBlur(mask_outline,(21,21),0)
            mask_outline = cv2.threshold(mask_blur,5,255,cv2.THRESH_BINARY)[1]
        return mask_outline

    def image_contour(self,img):
        contours,_=cv2.findContours(image=img,mode=cv2.RETR_TREE,method=cv2.CHAIN_APPROX_SIMPLE)

        area=0
        ctr = None
        for contour in contours:
            ctrarea = cv2.contourArea(contour)
            if ctrarea > area:
                area = ctrarea
                ctr = contour

        epsilon=0.001*cv2.arcLength(ctr,True)
        approx=cv2.approxPolyDP(ctr,epsilon,True)
        cv2.drawContours(self.img,[approx],-1,(30,225,30),2)
        cv2.drawContours(self.img,approx,-1,(0,255,0),10)
        return approx
    
    def get_center(self):

        x= int(self.img.shape[1]/2)
        y= -int(self.img.shape[0]/2)
        return [x,y]

    def image_show(self,img):
        cv2.imshow("window",img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

#TEST
# img_adress = '/home/bitcraze/Desktop/catkin_ws/src/app-client/fire/yngn2.jpg'
# imge = cv2.imread(img_adress)
# a= AppCv(imge)
# a.image_show(a.image)

#ROSTEST
# import appnode
# import time

# nd = appnode.AppNode()
# nd.node_start()
# nd.sub_image('image/image_topic')

# time.sleep(3)
# cv2.imshow("window",nd.image)
# cv2.waitKey(0)
# cv2.destroyAllWindows()