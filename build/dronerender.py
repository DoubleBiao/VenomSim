'''
==================
Rotating a 3D plot
==================

A very simple animation of a rotating 3D plot.

See wire3d_animation_demo for another simple example of animating a 3D plot.
'''

from mpl_toolkits.mplot3d import axes3d
import matplotlib.pyplot as plt
import numpy as np
from math import radians,sin,cos,degrees
import mpl_toolkits.mplot3d.art3d as art3d
from mpl_toolkits.mplot3d.art3d import Poly3DCollection, Line3DCollection
import io





class dronerender():
    def __init__(self):
        self.resolution = 20
        self.arm_color = ["r","g","b","y"]
        arm_len = 1.2
        rotor_radius = 0.8
        
        
        self.arm = np.array([[1,0,0],   # 4 endpoints and a center
                       [-1,0,0],
                       [0,1,0],
                       [0,-1,0],
                       [0,0,0]])*arm_len

        self.vert = self.generate_circle(arm_len,rotor_radius,self.resolution)
        self.cam_ori = [0,0,-90]
        self.focal_len = 10
        self.view_range = self.focal_len #- 5.5
                
    def generate_circle(self,arm_len,rotor_radius,resolution):
        theta = np.linspace(0,2*np.pi,resolution)
        theta = theta.reshape((len(theta),1))
        cx = np.cos(theta)
        cy = np.sin(theta)
        cz = theta * 0
        rotor_config = [[1,0],[-1,0],[0,1],[0,-1]]
        
        vert = []
        for rotorind in range(4):
            tmpx = cx*rotor_radius + rotor_config[rotorind][0]*(arm_len+rotor_radius)
            tmpy = cy*rotor_radius + rotor_config[rotorind][1]*(arm_len+rotor_radius)
            vert.append(np.concatenate((tmpx,tmpy,cz),axis = 1))
        return vert

    def get_coordinate(self,pos_hunter,o_hunter,pos_target,o_target):
        def getrotation(o):
            A = radians(o[2])
            B = radians(o[1])
            C = radians(o[0])
            R = np.array([[cos(A)*cos(B), cos(A)*sin(B)*sin(C)-sin(A)*cos(C), cos(A)*sin(B)*cos(C) + sin(A)*sin(C)],
                          [sin(A)*cos(B), sin(A)*sin(B)*sin(C)+cos(A)*cos(C), sin(A)*sin(B)*cos(C) - cos(A)*sin(C)],
                          [-sin(B),       cos(B)*sin(C),                      cos(B)*cos(C)                       ]])
    
            return R
        def transform_vert(rotation,transition,vert):
            vert = np.dot(vert,rotation.T)
            vert = vert + transition
            return vert

        R_tar = getrotation(o_target)
        R_hun = getrotation(o_hunter)
        R_cam = getrotation(self.cam_ori)
        world_trans = pos_target - pos_hunter

        out_rot = np.linalg.inv(np.dot(R_hun,R_cam))
        rotation = np.dot(out_rot,R_tar)
        transition = np.dot(out_rot,world_trans) + np.array([[self.focal_len],[0],[0]])

        vert_series= []
        for rotorind in range(4):
            output = transform_vert(rotation,transition.T,self.vert[rotorind])
            #output = vert[rotorind]
            vert_ind = [output[i] for i in range(self.resolution)]
            vert_series.append(vert_ind)

        arm_vert = transform_vert(rotation,transition.T,self.arm)
        return vert_series,arm_vert

    def rendering(self,rotor_vert,arm_vert):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.add_collection3d(Poly3DCollection(rotor_vert))
        for armind in range(4):
            ax.plot3D([arm_vert[armind,0],arm_vert[4,0]],[arm_vert[armind,1],arm_vert[4,1]],[arm_vert[armind,2],arm_vert[4,2]],linewidth = 5,color = self.arm_color[armind])  #arm1

##        ax.set_xlabel('x-axis')
##        ax.set_ylabel('y-axis')
##        ax.set_zlabel('z-axis')

        ax.set_xlim(left = -self.view_range,right = self.view_range)
        ax.set_ylim(bottom = -self.view_range,top = self.view_range)
        ax.set_zlim(bottom = -self.view_range,top = self.view_range)
        ax.view_init(0,0)
        ax.grid(False)
        ax.set_axis_off()
        
        buffer_ = io.BytesIO()
        plt.savefig(buffer_,format = 'png')
        buffer_.seek(0)
        outimg = plt.imread(buffer_)
        plt.close()

        return outimg
    
    def dorender(self,pos_hunter,o_hunter,pos_target,o_target):
        rotor_vert,arm_vert = self.get_coordinate(pos_hunter,o_hunter,pos_target,o_target)
        return   self.rendering(rotor_vert,arm_vert)

if __name__ == "__main__":
    o_hunter = [-45,0,0]
    o_target = [0,0,0]
    pos_hunter = np.array([[0],[0],[0]])
    pos_target = np.array([[0],[5],[5]])

    rendering = dronerender()
    output = rendering.dorender(pos_hunter,o_hunter,pos_target,o_target)
    plt.close('all')
    plt.imshow(output)
    plt.show()
