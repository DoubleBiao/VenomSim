from DroneSimEnv import *
import random
import time


def actiongenerator():
    return [random.random(),random.random(),random.random(),random.random()]


env = DroneSimEnv()
itetime = 100

for i in range(itetime):
    state, reward, done, dis = env.step([0,0,0,1])
    print("reward: ",reward)
    print("done: ",done)
    print("distance: ",dis['distance'])
    
    time.sleep(0.5)

env.stop()
