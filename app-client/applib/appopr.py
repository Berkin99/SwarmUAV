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

#   appopr.py

#------------------------------------------------------------------
# BU THREAD ÇALIŞIRKEN DİĞER THREATLER VERİ GÜNCELLEMESİ YAPAMIYOR
#------------------------------------------------------------------

#------------------------------------------------------------------
# ///////////////////////  OUT OF ORDER //////////////////////////
#------------------------------------------------------------------

import threading
import time
from math3d import*

class PolyOpr:
    def __init__(self,op_list=[]):
        self.start_time=time.time()
        self.operations = op_list
        self.state = self.poly2op(op_list[0])
        self.end = False
        operate = threading.Thread(target=self.poly_opr,daemon=True).start()
    
    def poly_opr(self):
        polyop = self.operations
        poly_time = self.start_time
        for i in range(polyop.__len__()):

            self.state = self.poly2op(polyop[i])
            if polyop[i][2][1] > 0:
                while (time.time()-poly_time)/polyop[i][2][1] < 1:
                    time.sleep(0.01)

            if i+1 >= polyop.__len__():continue
            operation =OprLerp([self.poly2op(polyop[i]),self.poly2op(polyop[i+1]),polyop[i][2][2]])
            while operation.end == False:
                self.state=operation.state
            poly_time = time.time()
        self.end = True
    
    def poly2op(self,op):
        return [op[0],op[1],op[2][0]]


class OprLerp:
    def __init__(self,operation=[[[0,0,0],[0,0,0],1],[[0,0,0],[0,0,0],1],0]):
        self.start_time = time.time()
        self.opt1 = operation[0]
        self.opt2 = operation[1]

        self.pos = operation[0][0]
        self.rot = operation[0][1]
        self.scale = operation[0][2]
        self.state = [self.pos,self.rot,self.scale]
        self.timer = operation[2]

        if self.timer > 0:
            self.end=False
            operate = threading.Thread(target=self.opr,daemon=True).start()
        else:
            self.end=True

    def opr(self):
        tlerp = 0
        while tlerp < 1: 
            tlerp = (time.time()-self.start_time)/self.timer
            self.pos = vround(vlerp(self.opt1[0],self.opt2[0],tlerp),3)
            self.rot = vround(vlerp(self.opt1[1],self.opt2[1],tlerp),3)
            self.scale = round(self.opt1[2]*(1-tlerp) + self.opt2[2]*(tlerp),3)
            self.state = [self.pos,self.rot,self.scale]
            time.sleep(0.01)
        self.pos = vround(self.opt2[0],3)
        self.rot = vround(self.opt2[1],3)
        self.scale = round(self.opt2[2],3)
        self.state = [self.pos,self.rot,self.scale]
        self.end=True
