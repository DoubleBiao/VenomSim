from DroneSimEnv import *
import matplotlib.pyplot as plt
import dronerender
import random
import time


def actiongenerator():
    return [random.random(),random.random(),random.random(),random.random()]


env = DroneSimEnv()
drone_render = dronerender.dronerender()

itetime = 1000

fig = plt.figure()
#ax = fig.add_subplot(111)

o_hunter = [-45,0,0]
o_target = [0,0,0]
pos_hunter = np.array([[0],[0],[0]])
pos_target = np.array([[0],[5],[5]])

start_time = time.time()

for i in range(itetime):
    state, reward, done, dis = env.step([1,0,0,0])
    
    pos_hunter,o_hunter,pos_target,o_target = env.get_pos()
    
    output = drone_render.dorender(pos_hunter,o_hunter,pos_target,o_target)
    #plt.close('all')

    plt.imshow(output)
    plt.draw()
    plt.pause(.05)
    
    #print("reward: ",reward)
    #print("done: ",done)
    #print("distance: ",dis['distance'])
    #print(pos_hunter)
    #print(pos_target)
    #print(o_hunter)
    #print(o_target)
    #time.sleep(0.01)

elapsed_time = time.time() - start_time
print("time elapsed:",elapsed_time)
env.stop()
