import numpy as np

FOV = 110
HOV = 63
H = 0.01

def getnearsize(F,H,FOVnear):
    F = np.cos(np.radians(F))
    H = np.cos(np.radians(H))
    D = np.matrix([[F + 1, F - 1],[H - 1, H + 1]])
    b = np.matrix([[4 - 4*F],[4 - 4*H]]) * FOVnear**2
    upandright = D.I*b
    up = np.sqrt(upandright[0,0])/2;
    right = np.sqrt(upandright[1,0])/2;
    return up,right

up,right = getnearsize(FOV,HOV,H)

print(up,right)
