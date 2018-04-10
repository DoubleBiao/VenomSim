from dronesim import *
import random
import time

siminit([1,2,0],[0,0,180],[4,6,0],[0,0,0],20,5,180,180)

def actiongenerator():
    return [random.random(),random.random(),random.random(),random.random()]


for t in range(100000):
    #roll,pitch,yaw,throttle = cmdfromkeyboard()
    #simcontrol(actiongenerator(),[0,0,0,0])
        
 
    simrun(5000000,actiongenerator(),[0,0,0,0])
    pos_hunter,ori_hunter,acc_hunter,pos_target,ori_target,acc_target,thrust = siminfo()
       

    if t%300 == 0:
        #renderer.render(pos_hunter,ori_hunter,pos_target,ori_target)
        print(pos_hunter[0],pos_hunter[1],pos_hunter[2])

