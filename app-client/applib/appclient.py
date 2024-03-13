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

#   appclient_1.2.py 

# - cflib.crtp.init_drivers(enable_debug_driver=False)

#------------------------------------------------------
# YANGIN FORMASYON GOREVI
#------------------------------------------------------

import tkinter as tk
import tkinter.ttk as ttk
import time
import cflib.crtp 
import cv2

import math3d
import appcom
import appopr
import appshape
import appnode
import appcv


app_name='appclient'

# Crazyflie Radio 
# uri_list=[]

uri_list = [
    'radio://0/125/2M/E7E7E7E7C1',
    'radio://0/125/2M/E7E7E7E7C2',
    'radio://0/125/2M/E7E7E7E7C3',
    'radio://0/115/2M/E7E7E7E7C4',
    'radio://0/115/2M/E7E7E7E7C5'
]
uri_list=[]

ros_list = [
    'ika_F1',
    'ika_F2',
    'ika_F3',
    'ika_F4'
]

# obs_ros_list=[]

# Obstacle Ros
obs_ros_list=[
    'obstacle_0',
    'obstacle_1',
]

# Fire image 
image_adress =  '/home/bitcraze/Desktop/catkin_ws/src/app-client/fire/yngn1.jpg'
img_ros = '/top/cam/image_raw'
img_rotate = 1 # 1 = 90CW, -1 = 90CCW,  2 = 180
img_flip = 0 # flip around y axis
img_scale = 0.00416 # pixel/centimeter

# --------------------------------

class Client:
    
    def __init__(self,name,uris,ross = []):
        
        self.name = name
        self.uris = uris
        self.ross = ross

        self.cflen =uris.__len__()
        self.rvlen =ross.__len__()

        #Takeoff Hold ----------
        self.all_connected=False
        #-----------------------

        #Operation -------------
        self.is_operating =False
        self.operation = None
        self.operation_shape = 0
        #-----------------------
        
        self.cf_operator = appcom.CfComOperator(self.uris)
        self.rv_operator = appcom.RvComOperator(self.ross)
        
        #-------------ROSNODE-------------
        self.ros_node = appnode.AppNode()
        self.ros_node.node_start()
        # self.ros_node.sub_obs(obs_ros_list)
        self.ros_node.sub_image(img_ros)
        #---------------------------------

        #---------------------------------
        self.set_gui()

        self.root.after(100,self._client_loop)
        self.root.mainloop()
        #Client end --------------------
        
        self.cf_operator.terminate_all()
        self.rv_operator.terminate_all()

    def set_focus(self,event=None):
        x,y=self.root.winfo_pointerxy()
        self.root.winfo_containing(x,y).focus()

    def set_gui(self):
        
        self.root = tk.Tk()

        
        self.style = ttk.Style(self.root)
        self.root.tk.call('source','/home/bitcraze/Desktop/catkin_ws/src/app-client/applib/azure/azure.tcl')
        self.root.tk.call('set_theme','light')

        
        self.style.configure('Bold.TButton',font=('Arial',10,'bold'))
        self.style.configure('Bold.TLabel',font=('Arial',10,'bold'))
        self.style.configure('Capital.TLabel',font=('Arial',12,'bold'))
        self.style.configure('Small.TEntry',font=('Arial',6,'bold'))
        self.style.configure('Bold.TLabelframe.Label',font=('Arial',10,'bold'))
        self.style.configure('Red.TFrame',bg='orange')

        self.root.title(self.name)
        self.root.resizable(width=False,height=False)
        self.root.bind("<1>",self.set_focus)

        self.rootframe = ttk.Frame(self.root,style='Red.TFrame')
        self.rootframe.pack(padx=10,pady=10)

        #root

        self.mainlabel = tk.Label(self.rootframe,text=' yeniay {}'.format(self.name),width=70,anchor='w',font='Arial 15 bold',background='#89A',foreground='#333')
        self.mainlabel.grid(row=0,column=0,columnspan=4,sticky="w",pady=5)

        self.timelabel = ttk.Label(self.rootframe,text='00:00:00',style="Bold.TLabel",font='Arial 10',background='#89A',foreground='#333')
        self.timelabel.grid(row=0,column=1,columnspan=4,padx=5,sticky="e")
        
        self.countlabel = ttk.Label(self.rootframe,text='IHA:{a} IKA:{b}'.format(a=self.cflen,b=self.rvlen),style="Bold.TLabel",foreground='gray')
        self.countlabel.grid(row=1,columnspan=4,padx=5,sticky="w")
        self.obslabel = ttk.Label(self.rootframe,text='ENGEL:0',style="Bold.TLabel",foreground='gray')
        self.obslabel.grid(row=1,columnspan=4,padx=5,sticky="e")

        #cf
        self.cfframe=ttk.Labelframe(self.rootframe,text='connection',style='Bold.TLabelframe')
        self.cfframe.grid(row=2,columnspan=4,sticky='news',pady=0)
        self.cfr=[]
        for i in range(self.cflen):
            self.cfr.append(CfFrame(self.cfframe,self.uris[i],i))
            self.cfr[i].grid(row=int(i/2)+1,column=int(i%2),padx=0,sticky='news')
        
        #ctrl panel
        self.appanel = ttk.Labelframe(self.rootframe,text='control panel',style="Bold.TLabelframe")
        self.appanel.grid(row=3,column=0,columnspan=4,sticky='news',pady=10)

        self.controlt = tk.StringVar()
        self.control1 = ttk.Entry(self.appanel,width=22,textvariable=tk.StringVar())
        self.control1.grid(row=0,column=0,sticky='nes',pady=2)

        self.apply_button = ttk.Button(self.appanel,width=24,text='Apply',style='Bold.TButton',command=self.apply_shape)
        self.apply_button.grid(row=0,column=1,sticky='nes',pady=2)

        
        self.start_button= ttk.Button(self.appanel,width=25,text='Start',style='Bold.TButton',command=self.start_operation)
        self.start_button.grid(row=0,column=2,sticky='nes',pady=2)
        
        self.stop_button= ttk.Button(self.appanel,width=24,text='Stop',style='Bold.TButton',command=self.stop_operation)
        self.stop_button.grid(row=0,column=3,sticky='nes',pady=2)
        
        #shape panel
        self.shpr=[]
        
        for i in range(1):
            self.shpr.append(FormFrame(self.rootframe,i))
            self.shpr[i].grid(row=int(i/4)+5,column=int(i%4),padx=0,sticky='ns')

        self.plus_buton = ttk.Button(self.rootframe,text='+',width=9,style='Bold.TButton',command=self.plus_form)
        self.plus_buton.grid(row=8,column=0,sticky='w',pady=5)

        self.mnus_buton = ttk.Button(self.rootframe,text='-',width=9,style='Bold.TButton',command=self.mnus_form)
        self.mnus_buton.grid(row=8,column=0,sticky='e',pady=5)

        self.tf_buton = ttk.Button(self.rootframe,text='Take Off',style='Bold.TButton',command=self.take_off_all,width=24)
        self.tf_buton.grid(row=9,column=0,sticky='news',pady=5)

        self.ld_buton = ttk.Button(self.rootframe,text='Land',style='Bold.TButton',command=self.land_all,width=24)
        self.ld_buton.grid(row=9,column=1,sticky='news',pady=5)

        self.force_buton = ttk.Button(self.rootframe,text='Force Land',style='Bold.TButton',command=self.force_land_all,width=24)
        self.force_buton.grid(row=9,column=2,sticky='news',pady=5)

        self.fire_button= ttk.Button(self.rootframe,text='Fire Alarm',style='Bold.TButton',command=self.fire_alarm,width=24)
        self.fire_button.grid(row=9,column=3,sticky='news',pady=5)

        #status panel
        self.statsframe=ttk.Labelframe(self.rootframe,text='status',style="Bold.TLabelframe")
        self.statsframe.grid(row=10,columnspan=12,sticky='news')
        
        self.stats = ttk.Label(self.statsframe,text='Waiting for connection...',font=('Consolas',10))
        self.stats.grid(row=0,column=0,padx=10,pady=5)

    def plus_form(self):
        
        i = self.shpr.__len__()
        if i>=12:return
        self.shpr.append(FormFrame(self.rootframe,i))
        self.shpr[i].grid(row=int(i/4)+5,column=int(i%4),padx=0,sticky='ns')
        
    def mnus_form(self):
        
        i= self.shpr.__len__()-1
        if i<=0:return
        self.shpr[i].destroy()
        del(self.shpr[i])

    def _client_loop(self):
        
        self.state_control()
        self.node_control()
        self.timelabel['text'] = str(time.strftime("%H:%M:%S"))
        self.root.after(200,self._client_loop)
    
    def _cfdriver(self):
        self.cf_operator.set_ncstacles()
        self.root.after(50,self._cfdriver)

    def state_control(self):
        status=True
        for i in range(self.cflen):
            self.cfr[i].set_status(self.cf_operator.controllers[i].connection_state)
            if self.cfr[i].is_connected ==False:
                status=False
        if status and not self.all_connected:
            self.stats['text'] = 'Crazyflies are connected!'
            self.all_connected = True
        
        elif self.all_connected and not status:
            self.stats['text'] = 'Crazyflies are disconnected. :('
            self.all_connected= False

    def node_control(self):
        self.obslabel['text'] = "ENGEL:" + str(self.ros_node.obstacle_list.__len__()) 

    def apply_shape(self):
        try:
            shape_no = appshape.Shape_names[str(self.control1.get())]
            self.operation_shape=shape_no
            if shape_no == 12:
                oprt = self.shpr[0].construct() 
                
                self.operation= [oprt[0],oprt[1],oprt[2][0]]
                
                self.cf_operator.set_operation(self.operation,self.operation_shape)
                self.stats['text'] = '///////// SWARM /////////'
                return
            
            oprt = self.shpr[0].construct() 
            self.operation = [oprt[0],oprt[1],oprt[2][0]]
            self.cf_operator.set_operation(self.operation,self.operation_shape)    
            self.stats['text'] = 'Shape applied : ' + str(self.control1.get())

        except KeyError:
            self.stats['text'] = 'Shape name error.'
            print('Shape name error.')

    def start_operation(self):
        return
        if self.is_operating:return
        operation_list = []
        for shp in self.shpr:
            operation_list.append(shp.construct())

        self.oprstate = appopr.PolyOpr(operation_list.copy())
        
        self.is_operating=True
        self.stats['text'] = 'Operation Start.'
        self.operate()
        
    def operate(self):
        return
        if self.is_operating:
            self.operation = self.oprstate.state
            self.cf_operator.set_operation(self.operation,self.operation_shape)
            
            if self.oprstate.end: 
                self.is_operating = False
                self.stats['text'] = 'Operation End.'
            self.root.after(200,self.operate)
    
    def stop_operation(self):
        self.is_operating = False
        self.stats['text'] = 'Operation Stop.'
    
    def take_off_all(self):
        if self.all_connected:
            pos = self.shpr[0].get_pos()
            z = pos[2]
            if pos[2] == 0: z = 0.3
            self.cf_operator.take_off_all(z)
            self._cfdriver()
            self.cf_operator.set_obstacles(self.ros_node.obstacle_list)
            self.stats['text'] = 'Taking off. Z = {}'.format(z)
        else:
            self.stats['text'] = 'All Crazyflies are not connected.'
            
    def land_all(self):
        self.is_operating = False
        self.cf_operator.land_all()
        self.stats['text'] = 'Landing... Z = 0.25'
    
    def force_land_all(self):
        self.is_operating = False
        self.cf_operator.force_land_all()
        self.stats['text'] = 'Force Land.'

    def fire_alarm(self):

        self.is_operating = False
       
        imge = self.ros_node.image
        try:
            if imge == None:
                self.stats['text'] = 'Fire image does not exist.'
                print('Fire image does not exist.') 
                return
        except ValueError:
            pass
        
        self.stats['text'] = " ///////////// FIRE ALARM ACTIVATED ///////////// "

        if img_flip == 1 : imge = cv2.flip(imge,1)
        if img_rotate == 1:imge = cv2.rotate(imge,cv2.ROTATE_90_CLOCKWISE)
        elif img_rotate == -1:imge = cv2.rotate(imge,cv2.ROTATE_90_COUNTERCLOCKWISE)
        elif img_rotate == 2: imge = cv2.rotate(imge,cv2.ROTATE_180)
        
        firecv = appcv.AppCv(imge)
        
        #cv data transformation
        cvcenter = firecv.get_center()
        cvcontour = firecv.get_contour(firecv.points)
        cvoutline = firecv.get_contour(firecv.outline_points)
        cvoutline2 = firecv.get_contour(firecv.outline_points2)
        
        firecontour  =  self.cv2orj(cvcontour,cvcenter)
        fireoutline  =  self.cv2orj(cvoutline,cvcenter)
        fireoutline2 =  self.cv2orj(cvoutline2,cvcenter)
        
        #contour data set
        self.rv_operator.set_polyline_formation(fireoutline2)
        self.rv_operator.set_polyline_obstacle(fireoutline)
        self.rv_operator.set_firecontour(firecontour)
        self.rv_operator.set_navigators()

        self.cf_operator.set_polyline_formation(fireoutline2)
        self.cf_operator.set_polyline_obstacle(fireoutline)
        self.cf_operator.set_firecontour(firecontour)
        self.cf_operator.set_navigators()

        #start navigation
        self.rv_operator.start_navigations()
        self.cf_operator.start_navigations()

    def cv2orj(self,cvcloud=[],cvcenter=[0,0]):
        #cv image to positioning center, by meter
        pcl=math3d.origin_set(cvcloud,cvcenter,z0=True)
        pcl=math3d.scale_set(pcl,[0,0],img_scale,z0=True)
        return pcl

class CfFrame(ttk.Labelframe):
    
    def __init__(self,parent,uri,id):
        super().__init__(parent,text="CF {}".format(id+1),style="Bold.TLabelframe")
        self.is_connected =False
        self.urilabel = tk.Label(self, text=uri,font='Helvetica 12',anchor='w',width=38)
        self.statlabel=tk.Label(self,text='disconnected',font='Helvetica 12',anchor='e',fg='grey',width=0)

        self.urilabel.grid(row=0,column=0,ipadx=10,padx=10,sticky='news')
        self.statlabel.grid(row=0,column=0,padx=5,sticky='e')
    
    def set_status(self,status):
        if status == 0:
            self.statlabel['text'] = "disconnected"
            self.statlabel['fg'] = 'grey'
            self.is_connected = False
        elif status == 1:
            self.statlabel['text'] = "connecting..."
            self.statlabel['fg'] = 'grey'
            self.is_connected = False
        elif status == 2:
            self.statlabel['text'] = "-connected-"
            self.statlabel['fg'] = "#5CC"
            self.is_connected =True

class FormFrame(ttk.Labelframe):
    def __init__(self,parent,id):
        super().__init__(parent,text="{}".format(id+1),style='Bold.TLabelframe')
        self.id = id
        self.pos=[]
        self.rot=[]
        self.stt=[]
        self._entry(self.pos,'Pos',0)
        self._entry(self.rot,'Rot',1)
        self._entry(self.stt,'Stt',2,['scale','timer1','timer2'])
    
    def _entry(self,lst,name,c,nlst=['x','y','z']):

        self.title  = ttk.Label(self,text=name,font=('arial',9))
        self.title.grid(row=0,column=c,padx=5)

        xentry = ttk.Entry(self,width=6,textvariable=tk.StringVar(value=nlst[0]),style='Small.TEntry',font=('arial',9))
        yentry = ttk.Entry(self,width=6,textvariable=tk.StringVar(value=nlst[1]),style='Small.TEntry',font=('arial',9))
        zentry = ttk.Entry(self,width=6,textvariable=tk.StringVar(value=nlst[2]),style='Small.TEntry',font=('arial',9))

        lst.append(xentry)
        lst.append(yentry)
        lst.append(zentry)

        xentry.grid(row=1,column=c,padx=3)
        yentry.grid(row=2,column=c,padx=3)
        zentry.grid(row=3,column=c,padx=3)

    def construct(self):
        pos  = self.get_info(self.pos)
        rot  = self.get_info(self.rot)
        stt  = self.get_info(self.stt)
        return [pos,rot,stt]

    def get_info(self,inf):
        info=[0,0,0]
        try:
            info[0] = float(inf[0].get())
            info[1] = float(inf[1].get())
            info[2] = float(inf[2].get())
        except ValueError:
            print('Info is not float number!')
        return info

    def get_pos(self):
        return self.get_info(self.pos)
    def get_rot(self):
        return self.get_info(self.rot)
    def get_stt(self):
        return self.get_info(self.stt)
        
if __name__ == '__main__':
    # print ('Initializing drivers...')
    # cflib.crtp.init_drivers(enable_debug_driver=False)
    cfclient=Client(app_name,uri_list,ros_list)