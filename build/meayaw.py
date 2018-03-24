from dronesim import *


siminit([1,2,0],[0,0,0],[4,6,0],[0,0,0],5,10,10086,90)
#renderer = visualdrone()
it = 0

last_pos = np.array([None,None,None])
hunteryaw_rec = []
targetyaw_rec = []
simrange = 1000

for it in range(simrange):
     #roll,pitch,yaw,throttle = cmdfromkeyboard()

     simrun(5000000,[0,0,1,0],[0,0,1,0])
     pos_hunter,ori_hunter,acc_hunter,pos_target,ori_target,acc_target,thrust = siminfo()
       

     if it%30 == 0:
     #   renderer.render(pos_hunter,ori_hunter,pos_target,ori_target)
        print(pos_hunter[0],pos_hunter[1],pos_hunter[2])
     #   it+=1
     #print(t)
     hunteryaw_rec.append(ori_hunter[2])
     targetyaw_rec.append(ori_target[2])
dronesimapi.simstop()

pl.plot(np.arange(simrange),np.array(hunteryaw_rec))
pl.plot(np.arange(simrange),np.array(targetyaw_rec))

pl.show()
