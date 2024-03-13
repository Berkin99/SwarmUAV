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

#   appshape.py

import math

Blank =[0,0,0]

Easy =[
    [-0.5,0,0],
    [0.5,0,0]
]

Triangle = [
    [0,0.5,0],
    [-0.4,-0.2,0],
    [ 0.4,-0.2,0]
]

Square = [
    [0.5,0.5,0],
    [-0.5,0.5,0],
    [-0.5,-0.5,0],
    [0.5,-0.5,0]
]

Pentagon = [
    [0.5,-0.7, 0],
    [-0.5,-0.7,0],
    [0.8,0.25, 0],
    [-0.8,0.25,0],
    [0,0.83,0],
]

Alpha = [
    [0.8,-0.6, 0],
    [-0.8,-0.6,0],
    [0.4,0.1, 0],
    [-0.4,0.1,0],
    [0,0.8,0],
]

Cube = [
    [-0.5,-0.5,-0.5],
    [-0.5, 0.5,-0.5],
    [ 0.5,-0.5,-0.5],
    [ 0.5, 0.5,-0.5],
    [-0.5,-0.5, 0.5],
    [-0.5, 0.5, 0.5],
    [ 0.5,-0.5, 0.5],
    [ 0.5, 0.5, 0.5]
]

Pyramid = [
    [-0.5,-0.5,-0.2],
    [-0.5, 0.5,-0.2],
    [ 0.5,-0.5,-0.2],
    [ 0.5, 0.5,-0.2],
    [0,0,0.3]
]

Prism3 = [
    [-0.4,-0.2,-0.5],
    [ 0.4,-0.2,-0.5],
    [0,0.5,-0.5],
    [-0.4,-0.2, 0.5],
    [ 0.4,-0.2, 0.5],
    [0,0.5, 0.5],
]

Prism4 = [
    [-0.5,-0.5,-0.5],
    [-0.5, 0.5,-0.5],
    [ 0.5,-0.5,-0.5],
    [ 0.5, 0.5,-0.5],
    [-0.5,-0.5, 0.5],
    [-0.5, 0.5, 0.5],
    [ 0.5,-0.5, 0.5],
    [ 0.5, 0.5, 0.5]
]

Prism5 = [
    [0.5,-0.7, -0.5],
    [-0.5,-0.7,-0.5],
    [0.8,0.25, -0.5],
    [-0.8,0.25,-0.5],
    [0,0.83,   -0.5],

    [0.5,-0.7, 0.5],
    [-0.5,-0.7,0.5],
    [0.8,0.25, 0.5],
    [-0.8,0.25,0.5],
    [0,0.83,   0.5]
]

Prism6 = [
    [0,1,-0.5],
    [0.86,0.5, -0.5],
    [-0.86,0.5,-0.5],
    [-0.86,-0.5,-0.5],
    [0.86,-0.5, -0.5],
    [0,-1,-0.5],

    [0,1,-0.5],
    [0.86,0.5, 0.5],
    [-0.86,0.5,0.5],
    [-0.86,-0.5,0.5],
    [0.86,-0.5, 0.5],
    [0,-1,0.5]
]

Cylinder = [
    [-0.5,-0.5,-0.7],
    [-0.5, 0.5,-0.7],
    [ 0.5,-0.5,-0.7],
    [ 0.5, 0.5,-0.7],
    [ 0.5,-0.5,0],
    [ 0.5, 0.5,0],
    [-0.5,-0.5, 0.7],
    [-0.5, 0.5, 0.7],
    [ 0.5,-0.5, 0.7],
    [ 0.5, 0.5, 0.7]
]

Shapes={
    0 : Easy,
    1 : Triangle,
    2 : Square,
    3 : Cube, 
    4 : Pyramid,
    5 : Prism3, 
    6 : Prism4,
    7 : Prism5,
    8 : Prism6,
    9 : Cylinder,
    10: Pentagon,
    11: Alpha,
}

Shape_names={
    'Easy'      :0,
    'Triangle'  :1,
    'Square'    :2,
    'Cube'      :3,
    'Pyramid'   :4,
    'Prism3'    :5,
    'Prism4'    :6,
    'Prism5'    :7,
    'Prism6'    :8,
    'Cylinder'  :9,
    'Pentagon'  :10,
    'Alpha'     :11,
    'Swarm'     :12
}

class Shape:

    def __init__(self):
        self.pos = Blank
        self.rot = Blank
        self.scale  = 1
        self.type   = 0
        self.set_shape(Blank,Blank,1,0)

    def set_shape(self,pos=Blank,rot=Blank,scale=1,type=0):
        self.pos=pos
        self.rot=rot
        self.scale=scale
        self.type=type
        self.abs_points = Shapes[self.type]
        self.points = self.get_shape()
    
    def rotater(self,pcloud,rot):
        pcl = []
        x = math.radians(rot[0])
        y = math.radians(rot[1])
        z = math.radians(rot[2])
        
        for p in pcloud:

            p = [p[0],(p[1]*math.cos(x))-(p[2]*math.sin(x)),(p[1]*math.sin(x))+(p[2]*math.cos(x))]
            p = [(p[0]*math.cos(y))+(p[2]*math.sin(y)),p[1],(p[2]*math.cos(y))-(p[0]*math.sin(y))]
            p = [(p[0]*math.cos(z))-(p[1]*math.sin(z)),(p[0]*math.sin(z))+(p[1]*math.cos(z)),p[2]]

            p[0]= round(p[0],5)
            p[1]= round(p[1],5)
            p[2]= round(p[2],5)

            pcl.append(p)
        
        return pcl

    def scaler(self,pcloud,scl):
        pcl = []
        for p in pcloud:
            p = [p[0]*scl,p[1]*scl,p[2]*scl]
            p[0]= round(p[0],5)
            p[1]= round(p[1],5)
            p[2]= round(p[2],5)
            pcl.append(p)
        return pcl
    
    def poser(self,pcloud,pos):
        pcl = pcloud
        for p in pcl:
            p[0]=round(pos[0]+p[0],5)
            p[1]=round(pos[1]+p[1],5)
            p[2]=round(pos[2]+p[2],5)
        return pcl

    def get_shape(self):
        pcl = self.abs_points
        pcl = self.rotater(pcl,self.rot)
        pcl = self.scaler(pcl,self.scale)
        pcl = self.poser(pcl,self.pos)
        return pcl
    