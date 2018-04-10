from dronesim import *

hunterpos = [0,-2,-2]
hunterori = [180,0,0]
targetpos = [0,0,0]
targetori = [-45,0,45]



installcamera([45,0,90],80,80, 0.01, 100.0,400.0,300.0)


u,w,area,inscreen = projection(hunterpos,hunterori,targetpos,targetori)

print(u,w,area,inscreen )

