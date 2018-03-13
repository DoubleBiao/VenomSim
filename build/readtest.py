import numpy as np
import cv2
from dronesim import *


SCR_WIDTH = 400
SCR_HEIGHT = 300

siminit([-10,0,10],[0,0,0],[0,0,0],[0,0,0],5,10,SCR_WIDTH,SCR_HEIGHT,1)
outimg = np.zeros((SCR_HEIGHT,SCR_WIDTH,3)).astype(np.uint8)

pos_hunter,ori_hunter,acc_hunter,pos_target,ori_target,acc_target,thrust = \
siminfo(outimg)
outimg = np.flipud(outimg)
outimg = cv2.cvtColor(outimg, cv2.COLOR_BGR2RGB)
#img = cv2.imread('windranger_fanart_by_rannero-dacg8ad.png')
cv2.imshow('image',outimg)
cv2.waitKey(0)
cv2.destroyAllWindows()
