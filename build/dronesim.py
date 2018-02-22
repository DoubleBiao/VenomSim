from ctypes import *
import numpy as np
import keyboard

class infoformat(Structure):
    _fields_ = [\
    ("posx",c_double),("posy",c_double),("posz",c_double),\
    ("velocityx",c_double),("velocityy",c_double),("velocityz",c_double),\
    ("accx",c_double),("accy",c_double),("accz",c_double),\
    ("thetax",c_double),("thetay",c_double),("thetaz",c_double),\

    ("posx_t",c_double),("posy_t",c_double),("posz_t",c_double),\
    ("velocityx_t",c_double),("velocityy_t",c_double),("velocityz_t",c_double),\
    ("accx_t",c_double),("accy_t",c_double),("accz_t",c_double),\
    ("thetax_t",c_double),("thetay_t",c_double),("thetaz_t",c_double),\
    ("thrust",c_double)\
        ]

#windows version interface 
dronesimapi = CDLL('./drone_sim.so')

#set input type

dronesimapi.siminit.argtype = [c_double,c_double,c_double,c_double,c_double,c_double,\
                               c_double,c_double,c_double,c_double,c_double,c_double,\
                               c_double,c_double]

dronesimapi.simcontrol.argtype = [c_double,c_double,c_double,\
                                  c_double,c_double,c_double]
dronesimapi.simrun.argtype  = [c_ulonglong]

#set output type
dronesimapi.siminfo.restype = POINTER(infoformat)


#interface warper:
def siminit(pos_hunter, ori_hunter, pos_target, ori_target,speed_upbound_hunter,speed_upbound_target):
    dronesimapi.siminit(c_double(pos_hunter[0]),c_double(pos_hunter[1]),c_double(pos_hunter[2]),\
                        c_double(ori_hunter[0]),c_double(ori_hunter[1]),c_double(ori_hunter[2]),\
                        c_double(pos_target[0]),c_double(pos_target[1]),c_double(pos_target[2]),\
                        c_double(ori_target[0]),c_double(ori_target[1]),c_double(ori_target[2]),\
                        c_double(speed_upbound_hunter),c_double(speed_upbound_target))

def simrun(period):
    # input : period time in second
    dronesimapi.simrun(c_ulonglong(period))

def simcontrol(huntercmd,targetcmd = None):
    if targetcmd:
        dronesimapi.simcontrol(c_double(huntercmd[0]),c_double(huntercmd[1]),c_double(huntercmd[2]),c_double(huntercmd[3]),\
                                       c_double(targetcmd[0]),c_double(targetcmd[1]),c_double(targetcmd[2]),c_double(targetcmd[3]))
    else:
        dronesimapi.simcontrol(c_double(huntercmd[0]),c_double(huntercmd[1]),c_double(huntercmd[2]),c_double(huntercmd[3]),\
                                       c_double(0),c_double(0),c_double(0),c_double(0))

def siminfo():
    outinfo = dronesimapi.siminfo()
    pos_hunter = np.array([outinfo.contents.posx,outinfo.contents.posy,outinfo.contents.posz])
    ori_hunter = np.array([outinfo.contents.thetax,outinfo.contents.thetay,outinfo.contents.thetaz])
    acc_hunter = np.array([outinfo.contents.accx,outinfo.contents.accy,outinfo.contents.accz])
    
    pos_target = np.array([outinfo.contents.posx_t,outinfo.contents.posy_t,outinfo.contents.posz_t])
    ori_target = np.array([outinfo.contents.thetax_t,outinfo.contents.thetay_t,outinfo.contents.thetaz_t])
    acc_target = np.array([outinfo.contents.accx_t,outinfo.contents.accy_t,outinfo.contents.accz_t])

    
    return pos_hunter,ori_hunter,acc_hunter,pos_target,ori_target,acc_target,outinfo.contents.thrust

def simstop():
    dronesimapi.simstop()
    
##    
def cmdfromkeyboard():
    rolldict = {'a':-1,'d':1}
    pitchdict = {'w':1,'s':-1}
    yawdict = {'q':-1,'e':1}
    throttledict = {'-':-1,'=':1}
    
    def checkkeyboard(keydict,default_val):
        for key in keydict.keys():
            if keyboard.is_pressed(key):
                return keydict[key]
        return default_val

    roll = checkkeyboard(rolldict,0)
    pitch = checkkeyboard(pitchdict,0)
    yaw = checkkeyboard(yawdict,0)
    throttle = checkkeyboard(throttledict,0)

    return roll,pitch,yaw,throttle        
    

class visualdrone():
    def __init__(self,viewrange = 50,arrowlen = 10):
        self.range = viewrange
        self.rawlen = self.range/arrowlen

        fig = pl.figure(0)
        self.axis3d = fig.add_subplot(111, projection='3d')

    def render(self,pos_hunter,ori_hunter,pos_target,ori_target):
        def Rot_bn(o):
            A =o[2]

            B =o[1]

            C =o[0]

            R = np.array([[np.cos(A)*np.cos(B), np.cos(A)*np.sin(B)*np.sin(C)-np.sin(A)*np.cos(C), np.cos(A)*np.sin(B)*np.cos(C) + np.sin(A)*np.sin(C)],

                          [np.sin(A)*np.cos(B), np.sin(A)*np.sin(B)*np.sin(C)+np.cos(A)*np.cos(C), np.sin(A)*np.sin(B)*np.cos(C) - np.cos(A)*np.sin(C)],

                          [-np.sin(B),       np.cos(B)*np.sin(C),                      np.cos(B)*np.cos(C)                       ]])

            return R
        
        def draw3d(ax, xyz, R):
            # We draw in ENU coordinates, R and xyz are in NED
            ax.scatter(xyz[0], xyz[1], xyz[2])
            ax.quiver(xyz[0], xyz[1], xyz[2], R[0, 0], R[1, 0], R[2, 0], pivot='tail', \
                    color='red')
            ax.quiver(xyz[0], xyz[1], xyz[2], R[0, 1], R[1, 1], R[2, 1], pivot='tail', \
                    color='green')
            ax.quiver(xyz[0], xyz[1], xyz[2], R[0, 2], R[1, 2], R[2, 2], pivot='tail', \
            color='blue')        
 
        self.axis3d.cla()
        draw3d(self.axis3d,pos_hunter, self.rawlen * Rot_bn(ori_hunter))
        draw3d(self.axis3d,pos_target, self.rawlen * Rot_bn(ori_target))
        
        self.axis3d.set_xlim(-self.range,self.range)
        self.axis3d.set_ylim(-self.range,self.range)
        self.axis3d.set_zlim(-self.range,self.range)
        self.axis3d.set_xlabel('x')
        self.axis3d.set_ylabel('y')
        self.axis3d.set_zlabel('z')
        
        pl.pause(0.00001)
        pl.draw()

##############test#######################
if __name__ == "__main__":
    import animation as ani
    import numpy as np
    from scipy import linalg as la
    import matplotlib.pyplot as pl
    import time


    def Rot_bn(phi,theta,psi):
        cphi = np.cos(phi)
        sphi = np.sin(phi)
        cthe = np.cos(theta)
        sthe = np.sin(theta)
        cpsi = np.cos(psi)
        spsi = np.sin(psi)

        Rx = np.array([[1,    0,      0], \
                       [0,  cphi,  sphi], \
                       [0, -sphi,  cphi]])

        Ry = np.array([[cthe,  0,  -sthe], \
                       [   0,  1,      0], \
                       [sthe,  0,   cthe]])

        Rz = np.array([[ cpsi,  spsi, 0], \
                       [-spsi,  cpsi, 0], \
                       [    0,    0, 1]])

        R = Rx.dot(Ry).dot(Rz)
        return R

    siminit([1,2,3],[0,0,0],[4,6,5],[0,0,0],5,10)
    renderer = visualdrone()
    it = 0

    last_pos = np.array([None,None,None])

    for t in range(10000):
        roll,pitch,yaw,throttle = cmdfromkeyboard()
        simcontrol([roll,pitch,yaw,throttle],[roll,pitch,yaw,throttle])
        
 
        simrun(5000000)
        pos_hunter,ori_hunter,acc_hunter,pos_target,ori_target,acc_target,thrust = siminfo()
       

        if it%30 == 0:
            renderer.render(pos_hunter,ori_hunter,pos_target,ori_target)
        it+=1
    dronesimapi.simstop()

