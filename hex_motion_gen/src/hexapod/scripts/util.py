import numpy as np
import PyKDL
import matplotlib.pyplot as plt
import math
from mpl_toolkits.mplot3d.axes3d import Axes3D
from matplotlib import animation

def rot_error(qd, qe):
    eta_d = qd[3]
    eta_e = qe[3]
    epsilon_d = qd[0:3].reshape(3,1)
    epsilon_e = qe[0:3].reshape(3,1)
    
    error = eta_e*epsilon_d - eta_d*epsilon_e - np.matmul(skew(epsilon_d),epsilon_e)
    return error
    
def skew(vector):
    """
    this function returns a numpy array with the skew symmetric cross product matrix for vector.
    the skew symmetric cross product matrix is defined such that
    np.cross(a, b) = np.dot(skew(a), b)

    :param vector: An array like vector to create the skew symmetric cross product matrix for
    :return: A numpy array of the skew symmetric cross product vector
    """
    if isinstance(vector, np.ndarray):
        return np.array([[0, -vector.item(2), vector.item(1)],
                         [vector.item(2), 0, -vector.item(0)],
                         [-vector.item(1), vector.item(0), 0]])
    else:
        return np.array([[0, -vector[2], vector[1]], 
                         [vector[2], 0, -vector[0]], 
                         [-vector[1], vector[0], 0]])


def quaternion_mul(qd,qe):
    eta_d = qd[3]
    eta_e = qe[3]
    epsilon_d = qd[0:3].reshape(3,1)
    epsilon_e = qe[0:3].reshape(3,1)

    cross = np.cross(epsilon_d.squeeze(),epsilon_e.squeeze()).reshape(3,1)

    eta_q_mul = eta_d * eta_e - np.matmul(np.transpose(epsilon_d),epsilon_e)
    epsilon_q_mul = eta_d*epsilon_e + eta_e*epsilon_d + cross

    print(eta_q_mul)
    print(epsilon_q_mul)

    q_mul = np.concatenate([epsilon_q_mul,eta_q_mul])

    return q_mul

def bezier_curve(p0,pf,v0,vf,T,t):

    alpha = vf + 2*v0 + (2/T)*(p0-pf)
    pt = (1-(t/T)*(t/T))*(p0+v0*t) + (t/T)*(t/T)*(pf+alpha*(t-T))
    vt = v0 - 2*t*p0/(T*T)-3*v0*(t/T)*(t/T) + 2*t*pf/(T*T) + 3*alpha*(t/T)*(t/T) - 2*alpha*t/T
    return pt, vt

def bezier_loop(p1,p2,p3,start_p,T1,T2,t):
    '''
    bezier loop:
                ____T1______ p2_____T1______
                |                          |   
                |                          |
                |p1__________T2__________p3|
    The loop consists 2 parts: p1-p2-p3 and p3-p1
    T1 is the time from p1 to p2 and p2 to p3
    T2 is the time from p3 to p2
    We have the relation: T2 = 2*T1
    start_p is the initial position of the loop        
    '''
    # v0=-0.05
    # if p2[2]-p1[2]>0:
    #     v=v0
    # else:
    #     v=-v0
    v=0
    t = t%(2*T1+T2)
    if start_p == 'p1':
        if t<T1:
            pt,vt = bezier_curve(p1,p2,np.array([0,v,0]),np.array([0,-v,0]),T1,t)
        elif t>=T1 and t<2*T1:
            pt,vt = bezier_curve(p2,p3,np.array([0,-v,0]),np.array([0,v,0]),T1,t-T1)
        else:
            pt,vt = bezier_curve(p3,p1,np.array([0,v,0]),np.array([0,v,0]),T2,t-2*T1)
            

    elif start_p == 'p3':
        if t<T2:
            pt,vt = bezier_curve(p3,p1,np.array([0,v,0]),np.array([0,v,0]),T2,t)
        elif t>=T2 and t<T2+T1:
            pt,vt = bezier_curve(p1,p2,np.array([0,v,0]),np.array([0,-v,0]),T1,t-T2)
        else:
            pt,vt = bezier_curve(p2,p3,np.array([0,-v,0]),np.array([0,v,0]),T1,t-T1-T2)
        pass
    
    return pt,vt

def compute_l2(l1,a):
    m = 2*a*l1-l1*l1
    l2_pos = -a+math.sqrt(m+a*a)
    l2_neg = -a-math.sqrt(m+a*a)
    return l2_pos, l2_neg

    pass

if __name__ == '__main__':
    import rospy
    import time
    rospy.init_node("util")
    p1a = np.array([0,       -0,       0])
    p2a = np.array([0,   -0.014,    0.02])
    p3a = np.array([0,   -0.014*2,     0])
    # p1b = np.array([0.5,    0,        0])
    # p2b = np.array([0.5,   -0.014, 0.02])
    # p3b = np.array([0.5,   -0.014*2,  0])
    T1 = 0.25
    T2 = 2*T1

    
    x_data_a = []
    y_data_a = []
    z_data_a = []

    x_data_b = []
    y_data_b = []
    z_data_b = []

    r = rospy.Rate(50)
    start_time = time.time()
    while not rospy.is_shutdown():
        t = time.time()-start_time
        pta,vta = bezier_loop(p1a,p2a,p3a,"p1",T1,T2,t)
        # ptb,vtb = bezier_loop(p1b,p2b,p3b,"p3",T1,T2,t)
        # pt, vt = bezier_curve(p3,p1,np.array([0,-0.1,0]),np.array([0,-0.1,0]),T1,t)

        x_data_a.append(pta[0])
        y_data_a.append(pta[1])
        z_data_a.append(pta[2])

        # x_data_b.append(ptb[0])
        # y_data_b.append(ptb[1])
        # z_data_b.append(ptb[2])

        if t>5:
            break

        r.sleep()


    fig = plt.figure()
    ax = Axes3D(fig)
    # ax.set_xlim3d([-0.02,0.02])
    # ax.set_ylim3d([-0.1,1])
    # ax.set_zlim3d([0,0.02])

    
    # point_b = ax.scatter(x_data_b[0],y_data_b[0],z_data_b[0],'*',color='blue')

    def update(i):
        ax.clear()
        
        ax.set_xlim3d([-0.04,0.04])
        ax.set_ylim3d([0,0.03])
        ax.set_zlim3d([0,0.02])
        ax.plot3D(x_data_a,y_data_a,z_data_a,color='blue')
        point1 = ax.scatter(x_data_a[i],y_data_a[i], z_data_a[i],color='red')
        # point2 = ax.scatter(x_data_b[i],y_data_b[i], z_data_b[i],color='blue')
        



    ani = animation.FuncAnimation(fig,update,frames=500,interval=20)
    plt.show()

    
    