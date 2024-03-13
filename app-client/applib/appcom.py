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

#   appcom.py 

# Error Posibilities:
# - Threads cannot see other threads changes

import time
import threading
import appshape
import appnav

from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from math3d import*
from appnode import*

class CfComOperator:
    
    def __init__(self, uris):
        self.controllers = []
        self._uris = uris
        self.tf_form = []
        for uri in uris:
            self.controllers.append(CfCom(uri))
        
    def is_arrived(self):
        for controller in self.controllers:
            if controller.arrived == False:
                return False
        return True

    def stop_point(self):
        pcl = self.get_positions()
        self.set_targets(pcl)

    def get_positions(self):
        pcloud = []
        for controller in self.controllers:
            pcloud.append(controller.est_pos)   
        return pcloud

    # ----------NAV------------

    def set_polyline_obstacle(self,pline):
        for controller in self.controllers:
            controller.pl_obstacle = pline

    def set_firecontour(self,pline):
        for controller in self.controllers:
            controller.firecontour = pline

    #???
    def set_polyline_formation(self,pline):
        # positioning center, by meter
        pcl = pline
        pcl.append(pline[0])
        formation = peucker2d(pcl,self.controllers.__len__())
        self.set_formation(formation)

    #???
    def set_formation(self,pcloud):
        # positioning center, by meter
        formation = pcloud.copy()
        for controller in self.controllers:
            if formation.__len__()==0:return
            formp = vnearest(p2d(controller.est_pos),pcloud,z0=True)
            formation.remove(formp)
            controller.nav_target = [formp[0],formp[1]]

    def set_navigators(self):
        for controller in self.controllers:
            controller.set_navigator()
    
    def start_navigations(self):
        for controller in self.controllers:
            controller.cf_state = controller.STATE_NAV
   
    # ----------NAV------------

    def set_operation(self,form,typ=0):
        if typ == 12:
            tff =[]
            for p in self.tf_form:
                tff.append([p[0],p[1],p[2]])
            formation = vposer(tff,form[0])
            self.set_targets(formation)
        else:    
            formation = appshape.Shape()
            formation.set_shape(form[0],form[1],form[2],typ)
            self.set_targets(formation.points)
    
    def set_targets(self,pcloud):
        print('TARGETS')
        print(pcloud)
        i=0
        for controller in self.controllers:
            if i >= pcloud.__len__():break
            controller.target_pos = pcloud[i]
            i = i+1

    def set_obstacles(self,obs_set):
        for controller in self.controllers:
            controller.obstacles = obs_set.copy()
    
    def set_ncstacles(self):
        ncs_set = self.get_positions()
        i=0
        for controller in self.controllers:
            ncs = ncs_set.copy()
            del(ncs[i])
            controller.ncstacles = ncs.copy()
            i=i+1

    def take_off_all(self,z=0.3):
        for controller in self.controllers:
            controller.take_off(z)

        tf_form = self.get_positions()
        tf_center = vort(tf_form)
        tf_f = origin_set(tf_form,tf_center,z0=True)
        self.tf_form=pl3d(tf_f.copy())
        print('FORMASYON')
        print(self.tf_form)

    def land_all(self):
        for controller in self.controllers:
            controller.land()
    
    def force_land_all(self):
        for controller in self.controllers:
            controller.force_land()
    
    def terminate_all(self):
        for controller in self.controllers:
            controller.terminate()

class CfCom:

    CS_DISCONNECTED = 0
    CS_CONNECTING = 1
    CS_CONNECTED = 2

    STATE_IDLE = 0
    STATE_TRACK = 1
    STATE_NAV = 2

    NAV_GRID_X = 128 # positioning x, self center
    NAV_GRID_Y = 128 # positioning y, self center
    NAV_ORIGIN = [64,64]
    NAV_RESOLUTION = 40

    def __init__(self,uri):
        self.uri = uri
        self.stay_alive = True

        self.logstart = False
        self.arrived = False
        self.obstacles =[]
        self.ncstacles =[]
        
        self.origin = appnav.NavOrigin(self.NAV_ORIGIN,self.NAV_RESOLUTION)
        self.trackpath = [] # positioning center, by meter
        self.pl_obstacle=[] # positioning center, by meter
        self.firecontour = [] # positioning center, by meter
        self.nav_target = [0,0]

        self.pad_pos = [0,0,0]
        self.est_pos = [0,0,0]
        self.target_pos = [0,0,0]
        
        self.reset()
        self.process_thread = threading.Thread(target=self.process)
        self.process_thread.start()
    
    def reset(self):
        self.connection_state = self.CS_DISCONNECTED
        self.cf_state = self.STATE_IDLE
        self._cf = None
        self._log_conf = None
        self._connection_hold = 0
        
    def process(self):
        while self.stay_alive:
            if self.connection_state == self.CS_DISCONNECTED:
                if time.time() > self._connection_hold:
                    self.connect()
            elif self.cf_state == self.STATE_TRACK:
                self.tracking_target()
            elif self.cf_state == self.STATE_NAV:
                self.navigate()

            time.sleep(0.1)
        self._cf.close_link()

    def connect(self):
        if self.connection_state != self.CS_DISCONNECTED:
            print("Can only connect when disconnected")
            return

        self.connection_state = self.CS_CONNECTING

        self._cf = Crazyflie(rw_cache='./cache')

        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)

        print("Connecting to " + self.uri)
        self._cf.open_link(self.uri)
    
    def is_connected(self):
        return self.connection_state == self.CS_CONNECTED

    def _connected(self, link_uri):
        self.connection_state = self.CS_CONNECTED
        print('Connected to %s' % link_uri)
        self._setup_log()

    def _connection_failed(self, link_uri, msg):
        print('Connection to %s failed: %s' % (link_uri, msg))
        self._set_disconnected(3)

    def _connection_lost(self, link_uri, msg):
        print('Connection to %s lost: %s' % (link_uri, msg))

    def _disconnected(self, link_uri):
        print('Disconnected from %s' % link_uri)
        self._set_disconnected()

    def _set_disconnected(self, hold_back_time=3):
        self.reset()
        self._connection_hold = time.time() + hold_back_time   

    def _setup_log(self):

        self._log_conf = LogConfig(name='Stabilizer', period_in_ms=50)
        self._log_conf.add_variable('stateEstimate.x', 'float')
        self._log_conf.add_variable('stateEstimate.y', 'float')
        self._log_conf.add_variable('stateEstimate.z', 'float')
        self._cf.log.add_config(self._log_conf)
        self._log_conf.data_received_cb.add_callback(self._log_data)
        self._log_conf.start()
    
    def _log_data(self, timestamp, data, logconf):

        self.est_x = data['stateEstimate.x']
        self.est_y = data['stateEstimate.y']
        self.est_z = data['stateEstimate.z']
        self.est_pos=[self.est_x,self.est_y,self.est_z]

        if self.logstart==False:
            self.pad_pos = [self.est_x,self.est_y,0.20]
            self.logstart = True

    #------------- NAVIGATION ---------------
    
    #???
    def set_navigator(self):
        navpos = [p2d(self.est_pos),p2d(self.nav_target)]
        navpos = self.origin.orj2nav(navpos)
        self.navigator = appnav.Navigator([self.NAV_GRID_X,self.NAV_GRID_Y],navpos[0],navpos[1])
        navigator_plobstacle = self.origin.orj2nav(self.pl_obstacle)
        self.navigator.set_polyline(navigator_plobstacle)
        self.navigator.nav_start()
        self.navpath = []
        npr = self.navigator.navpath
        npr = self.origin.nav2orj(npr)
        
        for p in npr:
            p=[round(p[0],2),round(p[1],2)]
            self.navpath.append(p)
        self.trackpath=[]
        self.trackpath = peucker2d(self.navpath,7)
        self.trackpath.reverse()
        if self.navpath.__len__()>1:self.trackpath.append(self.navpath[0])

        if self.firecontour != None:
            fire_end = vnearest(self.navpath[0],self.firecontour,z0=True)
            self.trackpath.append([round(fire_end[0],2),round(fire_end[1],2)]) 
        
        print('CF_{}'.format(self.adress))
        print(self.trackpath)

    #???
    def navigate(self):
        try:
            if self.trackpath == None : 
                self.cv_state = self.STATE_TRACK
                return
        except ValueError:pass

        if self.trackpath.__len__()<=0:
            self.cv_state = self.STATE_TRACK
            print(self.uri + ' navigation end')
            return
        self.target_pos = [self.trackpath[0][0],self.trackpath[0][1],1]
        self.tracking_target()
        
        if vdist(self.est_pos,[self.trackpath[0][0],self.trackpath[0][1],1])<0.2:
            del(self.trackpath[0])

    #------------- NAVIGATION ---------------

    def tracking_target(self):
        if vdist(self.est_pos,self.target_pos)<0.3:
            self.arrived = True
        else:
            self.arrived = False
        
        main_f = vsub(self.target_pos,self.est_pos)
        main_f = vmax(main_f,1)

        react_f=[0,0,0]

        # -------------------REACTFORCE----------------------
        # # xyz react force
        # if self.obstacles.__len__()>0:
        #     obs = vnearest(self.est_pos,self.obstacles)
        #     obs_dist = vdist(self.est_pos,obs)-0.05
        #     if obs_dist<=0:obs_dist=0.01
        #     if obs_dist < 0.25:
        #         kd = 0.1/(obs_dist**2)
        #         obs_f = vmult(vsub(self.est_pos,obs),kd)
        #         react_f = vadd(react_f,obs_f)
        
        # --------------------------------------------------
        # xy
        est_p = [self.est_x,self.est_y,0]
        if self.obstacles.__len__()>0:
            obs = vnearest(est_p,self.obstacles,z0=True)
            obs = p3d(obs)
            obs_dist = vdist(obs,est_p)-0.07
            if obs_dist<=0:obs_dist=0.01
            if obs_dist < 0.35:
                kd = 0.08/(obs_dist**2)
                obs_f = vmult(vsub(est_p,obs),kd)
                react_f = vadd(react_f.copy(),obs_f)
        # --------------------------------------------------
        # xyz collision avoidance

        if self.ncstacles.__len__()>0:
            ncf = vnearest(self.est_pos,self.ncstacles)
            ncf_dist = vdist(self.est_pos,ncf)-0.04
            if ncf_dist<=0:ncf_dist=0.01
            if ncf_dist < 0.20:
                kd = 0.05/(ncf_dist**2)
                ncf_f = vmult(vsub(self.est_pos,ncf),kd)
                react_f = vadd(react_f.copy(),ncf_f)

        # ------------------ ALL FORCE --------------------

        # alstacles = self.ncstacles + self.obstacles

        # if alstacles.__len__()>0:
        #     alf = vnearest(self.est_pos,alstacles)
        #     alf_dist = vdist(self.est_pos,alf)-0.05
        #     if alf_dist<=0:alf_dist=0.01
        #     if alf_dist < 0.25:
        #         kd = 0.1/(alf_dist**2)
        #         alf_f = vmult(vsub(self.est_pos,alf),kd)
        #         react_f = vadd(react_f.copy(),alf_f)

        # --------------------------------------------------

        react_f = vmax(react_f.copy(),1.25)
        #(FT + FN)
        net_f = vadd(main_f,react_f)
        net_f = vround(vmax(net_f,0.3),5)
        self._cf.commander.send_velocity_world_setpoint(net_f[0],net_f[1],net_f[2],0)

    def take_off(self,z=0.3):
        if self.cf_state != self.STATE_IDLE:return
        if self.connection_state == self.CS_CONNECTED:
            self.target_pos = [self.est_x,self.est_y,z]
        self.cf_state = self.STATE_TRACK
    
    def land(self):
        if self.cf_state == self.STATE_IDLE:return
        if self.connection_state == self.CS_CONNECTED:
            self.target_pos = [self.est_x,self.est_y,self.pad_pos[2]]
        self.cf_state = self.STATE_TRACK

    def force_land(self):
        self.cf_state = self.STATE_IDLE
        if self.connection_state == self.CS_CONNECTED:
            self._cf.commander.send_stop_setpoint()
        
    def terminate(self):
        self.stay_alive = False


class CfSwarm:
    
    def __init__(self,swarm_len):
        self.positions=[]
        for i in range(swarm_len):
            self.positions.append([0,0,0])
    
    def set_pos(self,adress,pos):
        self.positions[adress] = pos

    def get_others(self,adress):
        others = self.positions.copy()
        del(others[adress])
        return others
    
    def get_nearest(self,adress):
        pos = self.positions[adress]
        others = self.get_others(adress)
        return vnearest(pos,others)

#---------------------------------------


class RvComOperator:
    
    def __init__(self,uris):
        self.controllers = []
        self._uris = uris
        for uri in uris:
            self.controllers.append(RvCom(uri))
    
    def stop_point(self):
        for controller in self.controllers:
            controller.rv_state = controller.STATE_IDLE

    def set_polyline_obstacle(self,pline):
        # positioning center, by meter
        for controller in self.controllers:
            controller.pl_obstacle = pline
    
    def set_polyline_formation(self,pline):
        # positioning center, by meter
        pcl = pline
        pcl.append(pline[0])
        formation = peucker2d(pcl,self.controllers.__len__())
        self.set_formation(formation)

    def set_formation(self,pcloud):
        # positioning center, by meter
        formation = pcloud
        for controller in self.controllers:
            if formation.__len__()==0:return
            formp = vnearest(controller.est_pos,pcloud,z0=True)
            formation.remove(formp)
            controller.target_pos = formp
    
    def set_firecontour(self,pcloud):
        for controller in self.controllers:
            controller.firecontour = pcloud

    def set_navigators(self):
        for controller in self.controllers:
            controller.set_navigator()
    
    def start_navigations(self):
        for controller in self.controllers:
            controller.rv_state = controller.STATE_NAV

    def terminate_all(self):
        for controller in self.controllers:
            controller.terminate()

class RvCom:
    
    STATE_IDLE = 0
    STATE_TRACK = 1
    STATE_NAV = 2

    NAV_GRID_X = 128 # positioning x, self center
    NAV_GRID_Y = 128 # positioning y, self center
    NAV_ORIGIN = [64,64]
    NAV_RESOLUTION = 40

    def __init__(self,uri):
        self.uri = uri
        self.stay_alive = True
        
        self._setup_log()
        self.reset()
        
        self.process_thread = threading.Thread(target=self.process)
        self.process_thread.start()
    
    def reset(self):
        self.rv_state = self.STATE_IDLE
        self.est_pos=[0,0] # positioning center, by meter
        self.target_pos = [0,0] # positioning center, by meter
        self.trackpath = None
        self.pl_obstacle=[] # positioning center, by meter
        self.firecontour = None # positioning center, by meter

        self.origin = appnav.NavOrigin(self.NAV_ORIGIN,self.NAV_RESOLUTION)

    def process(self):
        while self.stay_alive:
            if self.rv_state == self.STATE_TRACK:
                self.tracking_target()
            elif self.rv_state == self.STATE_NAV:
                self.navigate()
            time.sleep(0.1)
   
    def _setup_log(self):
        pos_adress = self.uri + '/position'
        set_adress = self.uri + '/goal_pose'
        acc_adress = self.uri + '/accomplish'
        self.ros_goal_pose = rospy.Publisher(set_adress,Point,queue_size=10)
        rospy.init_node('app_node',anonymous=True)
        rospy.Subscriber(pos_adress,Pose2D,self.log_pos)
        rospy.Subscriber(acc_adress,String,self.log_acc)
    
    def log_pos(self,data):
        self.est_pos = [data.x,data.y] # positioning center, by meter
    
    def log_acc(self,data):
        #DISCARD
        #del(self.trackpath[0])
        pass

    def set_navigator(self):
        navpos = [self.est_pos,self.target_pos]
        navpos = self.origin.orj2nav(navpos)
        self.navigator = appnav.Navigator([self.NAV_GRID_X,self.NAV_GRID_Y],navpos[0],navpos[1])
        navigator_plobstacle = self.origin.orj2nav(self.pl_obstacle)
        self.navigator.set_polyline(navigator_plobstacle)
        self.navigator.nav_start()
        self.navpath = []
        npr = self.navigator.navpath
        npr = self.origin.nav2orj(npr)
        
        for p in npr:
            p=[round(p[0],2),round(p[1],2)]
            self.navpath.append(p)
        self.trackpath=[]
        self.trackpath = peucker2d(self.navpath,7)
        self.trackpath.reverse()
        if self.navpath.__len__()>1:self.trackpath.append(self.navpath[0])

        if self.firecontour != None:
            fire_end = vnearest(self.navpath[0],self.firecontour,z0=True)
            self.trackpath.append([round(fire_end[0],2),round(fire_end[1],2)]) 
        
        print(self.uri)
        print(self.trackpath)
    
    
    def navigate(self):
        path = self.trackpath
        try:
            if path == None : 
                self.rv_state = self.STATE_IDLE
                return
        except ValueError:pass

        if path.__len__()<=0:
            self.rv_state = self.STATE_IDLE
            print(self.uri + ' navigation end')
            return
        self.target_pos = path[0]
        self.tracking_target()
        
        # CHANGED
        # if vdist(p3d(self.est_pos),p3d(self.trackpath[0]))<0.1:
        #     del(self.trackpath[0])
       
        # ---- TESTMODE ----
        del(self.trackpath[0])
        # -----------------

    def tracking_target(self):
        targetmsg = Point()
        targetmsg.x = float(self.target_pos[0])
        targetmsg.y = float(self.target_pos[1])
        targetmsg.z = 0.0
        self.ros_goal_pose.publish(targetmsg)

    def terminate(self):
        self.stay_alive = False
