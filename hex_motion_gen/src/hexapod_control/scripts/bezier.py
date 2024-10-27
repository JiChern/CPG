import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation
from utils import get_phase_list


class Bezier2(object):
    def __init__(self):
        self.x_pos = []
        self.y_pos = []


    def setPoint(self, x_vec, y_vec):
        self.x_pos = x_vec
        self.y_pos = y_vec


    def getPos(self, t):

        n_points = len(self.x_pos)

        #prevent the variables allocates at the same location
        x_cal, y_cal = self.x_pos.copy(), self.y_pos.copy()

        # x_cal, y_cal = [], []

        # for x in self.x_pos: x_cal.append(x)
        # for y in self.y_pos: y_cal.append(y)

        # print('x_cal in bezier 2:',self.x_pos, ' y_cal in bezier 2:', self.y_pos)


        for i in range(n_points-1):
            for j in range(n_points-i-1):
               x_cal[j] = (1.0-t)*x_cal[j] + t*x_cal[j+1]
               y_cal[j] = (1.0-t)*y_cal[j] + t*y_cal[j+1]

        x_ret = x_cal[0]
        y_ret = y_cal[0]
    

        return x_ret, y_ret

class Bezier(object):
    def __init__(self):
        self.x_pos = []
        self.y_pos = []


    def addPoint(self, x, y):
        self.x_pos.append(x)
        self.y_pos.append(y)



    def getPos(self, t):

        n_points = len(self.x_pos)

        #prevent the variables allocates at the same location
        x_cal, y_cal = [], []
        for x in self.x_pos: x_cal.append(x)
        for y in self.y_pos: y_cal.append(y)


       

        for i in range(n_points-1):
            for j in range(n_points-i-1):
               x_cal[j] = (1.0-t)*x_cal[j] + t*x_cal[j+1]
               y_cal[j] = (1.0-t)*y_cal[j] + t*y_cal[j+1]

        x_ret = x_cal[0]
        y_ret = y_cal[0]
        return x_ret, y_ret






if __name__ == "__main__":
    bezier = Bezier()
    # bezier.addPoint(-1,0)
    # bezier.addPoint(0,3.2)
    # bezier.addPoint(1,0)

    bezier.addPoint(-0.02,0)
    bezier.addPoint(0,0.02)
    bezier.addPoint(0.02,0)



    bezier4 = Bezier2()
    x1 = -0.02
    x2 = (-0.02 + 0.02)/2
    x3 = 0.02

    y1 = 0
    y2 = 0.04
    y3 = 0

    bezier4.setPoint(x_vec = [x1,x2,x3], y_vec=[y1,y2,y3])
    
    t = 0
    x_list = []
    y_list = []
    t_list = []
    for i in range(100):
    #     walker.step(0.005)
        t += 0.01
        x,y = bezier4.getPos(t)
        # print(bezier.x_pos)
        t_list.append(t)
        x_list.append(x)
        y_list.append(y)


    # t = 0
    # x_list2 = []
    # y_list2 = []
    # for i in range(100):
    # #     walker.step(0.005)
    #     t += 0.01
    #     x,y = bezier.getPos(t)
    #     # print(bezier.x_pos)
    #     x_list2.append(x)
    #     y_list2.append(y)



    plt.figure()
    plt.plot(t_list,y_list)
    # plt.plot(x_list,y_list)
    plt.show()

