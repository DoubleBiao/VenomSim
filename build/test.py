from dronesim import *

siminit([1,2,0],[0,0,180],[4,6,0],[0,0,0],5,10,180,180)  # init simulation session
renderer = visualdrone()                                 # init visulization
it = 0

for t in range(1000):
    simrun(5000000,[0,0,0,1])                 # run a step of simulation
    pos_hunter,ori_hunter,acc_hunter,pos_target,ori_target,acc_target,thrust,spd_hunter,spd_target = siminfo() # fetch the data
    print(spd_hunter,spd_target)
    if it%30 == 0:
        renderer.render(pos_hunter,ori_hunter,pos_target,ori_target)
        print(pos_hunter[0],pos_hunter[1],pos_hunter[2])
    it+=1
dronesimapi.simstop()                           # stop the simulation

