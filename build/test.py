from DroneSimEnv import *
import random
import time


def actiongenerator():
    return [random.random(),random.random(),random.random(),random.random()]


env = DroneSimEnv()
itetime = 1000

for i in range(itetime):
    state, reward, done, dis = env.step([1,0,0,1])
    #print("reward: ",reward)
    #print("done: ",done)
    #print("distance: ",dis['distance'])
    
    time.sleep(0.01)

env.stop()
