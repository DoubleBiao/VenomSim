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
                               c_double,c_double,c_double,c_double,c_double,c_double]

dronesimapi.simcontrol.argtype = [c_double,c_double,c_double,\
                                  c_double,c_double,c_double]
dronesimapi.simrun.argtype  = [c_ulonglong]

#set output type
dronesimapi.siminfo.restype = POINTER(infoformat)


#interface warper:
def siminit(pos_hunter, ori_hunter, pos_target, ori_target):
    dronesimapi.siminit(c_double(pos_hunter[0]),c_double(pos_hunter[1]),c_double(pos_hunter[2]),\
                        c_double(ori_hunter[0]),c_double(ori_hunter[1]),c_double(ori_hunter[2]),\
                        c_double(pos_target[0]),c_double(pos_target[1]),c_double(pos_target[2]),\
                        c_double(ori_target[0]),c_double(ori_target[1]),c_double(ori_target[2]),)

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
    rolldict = {'a':ord('+'),'d':ord('-')}
    pitchdict = {'w':ord('+'),'s':ord('-')}
    yawdict = {'q':ord('+'),'e':ord('-')}
    throttledict = {'+':ord('+'),'-':ord('-')}
    
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

    #dronesimapi.siminit(1,2,3,4,5,6,
    #                    1,2,3,4,5,6)

    siminit([1,2,3],[3,4,5],[4,6,5],[1,2,3])
    
    quadcolor = ['k']
    fig = pl.figure(0)
    axis3d = fig.add_subplot(111, projection='3d')
    pl.figure(0)

    it = 0

    for t in range(10000):
        roll,pitch,yaw,throttle = cmdfromkeyboard()
        #print(roll,pitch,yaw,throttle)
        #dronesimapi.simcontrol(c_char(roll),c_char(pitch),c_char(yaw),c_char(throttle),c_char(0),c_char(0),c_char(0),c_char(0))
        simcontrol([roll,pitch,yaw,throttle])
        
        #dronesimapi.simrun(c_ulonglong(5000000))
        simrun(5000000)
        #outinfo = dronesimapi.siminfo()
        pos_hunter,ori_hunter,acc_hunter,pos_target,ori_target,acc_target,thrust = siminfo()
        #print("posisiton:",outinfo.contents.posx,outinfo.contents.posy,outinfo.contents.posz)

        if it%30 == 0:
            axis3d.cla()
            ani.draw3d(axis3d,pos_hunter,Rot_bn(ori_hunter[0],ori_hunter[1],ori_hunter[2]), quadcolor[0])
            print(thrust)
            #ani.draw3d(axis3d,[outinfo.contents.posx_t,outinfo.contents.posy_t,outinfo.contents.posz_t], Rot_bn(outinfo.contents.thetax_t,outinfo.contents.thetay_t,outinfo.contents.thetaz_t), quadcolor[0])
            ani.draw3d(axis3d,pos_target, Rot_bn(ori_target[0],ori_target[1],ori_target[2]), quadcolor[0])
            axis3d.set_xlim(-10, 10)
            axis3d.set_ylim(-10, 10)
            axis3d.set_zlim(0, 15)
            axis3d.set_xlabel('South [m]')
            axis3d.set_ylabel('East [m]')
            axis3d.set_zlabel('Up [m]')
            pl.pause(0.0001)
            pl.draw()
            #print("aaa")

        it+=1

    dronesimapi.simstop()


