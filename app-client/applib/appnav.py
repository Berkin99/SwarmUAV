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

#   appnav.py 

import math
import matplotlib.pyplot as plt
from math3d import*

class NavGrid:
    
    def __init__(self,x,y):
        self.gridx = int(x)
        self.gridy = int(y)
        #self.visual=NavVis(self.gridx,self.gridy)
        self.nodes= []
        self.target=None

        for gx in range(self.gridx):
            nd=[]
            for gy in range(self.gridy):
                nd.append(NavNode(gx,gy))
            self.nodes.append(nd)
    
    def set_polyline(self,pcloud,close=True):
        pn=None

        for p in pcloud:
            if pn == None:
                pn=p
            else:
                self.set_line(pn,p)
                pn = p
        if close: self.set_line(pn,pcloud[0])
    
    def set_line(self,p1,p2):
        _p1 = self.control(p1)
        _p2 = self.control(p2)

        dx = _p2[0]-_p1[0]
        dy = _p2[1]-_p1[1]
        dmax = int(max(math.fabs(dx),math.fabs(dy)))

        if dmax<=0:return

        for i in range((dmax+1)):
            ax = _p1[0]+int(dx*i/dmax)
            ay = _p1[1]+int(dy*i/dmax)

            self.set_obs([ax,ay])

    def set_obs(self,p):

        for x in range(2):
            for y in range(2):
                _p = self.control([p[0]+x-1,p[1]+y-1])
                self.nodes[_p[0]][_p[1]].obs=True
                #self.visual.set_color(_p[0],_p[1])
    
    def set_start(self,p):
        _p = self.control(p)
        if self.nodes[_p[0]][_p[1]].obs==False:
            #self.visual.set_color(_p[0],_p[1],[60,250,240])
            self.start = self.nodes[_p[0]][_p[1]]
   
    def set_target(self,p):
        _p = self.control(p)

        #self.visual.set_color(_p[0],_p[1],[255,60,60])
        self.target = self.nodes[_p[0]][_p[1]]
        self.target.target=True
        nbr = self.get_neighbours(self.target)
        for n in nbr:
            n.target = True
            #self.visual.set_color(n.pos[0],n.pos[1],[255,60,60])

    def get_distance(self,n1,n2):
        dx = math.fabs(n1.x-n2.x)
        dy = math.fabs(n1.y-n2.y)

        if dx>dy:
            return 14*dy+10*(dx-dy)
        return 14*dx + 10*(dy-dx)
            
    def get_neighbours(self,n):
        neighbour = []   
        
        for x in range(3):
            for y in range(3): 
                _p = self.control([n.x+x-1,n.y+y-1])
                if _p == n.pos : continue  
                neighbour.append(self.nodes[_p[0]][_p[1]])
        return neighbour
   

    def control(self,p):
        px=p[0]
        py=p[1]

        if p[0] >= self.gridx: px = self.gridx-1
        elif p[0] < 0: px=0 

        if p[1]>=self.gridy: py = self.gridy-1
        elif p[1] < 0: py = 0

        return [int(px),int(py)]

class Navigator(NavGrid):
    
    def __init__(self,grid,start,target):
        super().__init__(grid[0],grid[1])
        self.set_start(start)
        self.set_target(target)
        self.open =[]
        self.closed = []
        self.open.append(self.start)
        self.navpath=[]

    def nav_start(self):
        while True:
            current = None
            for nd in self.open:
                if current == None or nd.get_fcost()<current.get_fcost():
                    current = nd
            
            if self.open.__contains__(current):
                self.open.remove(current)
            else:
                break

            self.closed.append(current)

            if current.target == True: 
                self.path = self.retrace_path(self.start,current)
                break

            neighbour = self.get_neighbours(current)

            for n in neighbour:
                if n.obs==True or self.closed.__contains__(n): continue

                virt_g = current.gcost + self.get_distance(current,n)
                
                if virt_g < n.gcost or not self.open.__contains__(n):
                    n.gcost = virt_g
                    n.hcost = self.get_distance(n,self.target)
                    n.parent = current
                    
                    if not self.open.__contains__(n):
                        self.open.append(n)
                        
    def retrace_path(self,start_n,end_n):
        path = []
        navpath = []
        current = end_n
        while current != start_n:
            path.append(current)
            navpath.append([current.x,current.y])
            current = current.parent
            #self.visual.set_color(current.x,current.y,[140,190,150])
        self.navpath = navpath
        return path
 
    # def show(self):
    #     self.visual.show()
        
class NavNode:

    def __init__(self,x,y):
        self.parent = None
        self.obs = False
        self.target = False
        self.pos = [x,y]
        self.x = x
        self.y = y
        self.gcost = 0
        self.hcost = 0
    
    def get_fcost(self):
        self.fcost = self.gcost+self.hcost
        return self.fcost

class NavVis:
    def __init__(self,x,y):
        self.gridx = int(x)
        self.gridy = int(y)
        self.pixels= []

        transr=True
        for gy in range(self.gridy):
            py=[]
            for gx in range(self.gridx):
                if transr:
                    py.append([250,250,250])
                    transr=False
                else:
                    py.append([240,240,240])
                    transr=True
            self.pixels.append(py)

    def set_color(self,x,y,color=[0,0,0]):
        self.pixels[self.gridy-y-1][x] = color
    
    def show(self):
        c = plt.imshow(self.pixels)
        plt.show()


class NavOrigin:
    
    def __init__(self,origin,res):
        self.NAV_ORIGIN = origin
        self.NAV_RESOLUTION = res

    def orj2nav(self,pcloud=[]):
        negorigin = [-self.NAV_ORIGIN[0],-self.NAV_ORIGIN[1]]
        pcl = scale_set(pcloud,[0,0],self.NAV_RESOLUTION,z0=True)
        pcl = origin_set(pcl,negorigin,z0=True)
        return pcl
    
    def nav2orj(self,pcloud=[]):
        pcl = origin_set(pcloud,[self.NAV_ORIGIN[0],self.NAV_ORIGIN[1]],z0=True)
        pcl = scale_set(pcl,[0,0],1/self.NAV_RESOLUTION,z0=True)
        return pcl

#TEST
# n = Navigator([81,81],[1,1],[40,40])
# n.set_line([0,40],[40,15])
# n.nav_start()
# print(n.navpath)