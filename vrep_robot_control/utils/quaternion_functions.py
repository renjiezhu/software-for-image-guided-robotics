#file containing basic quaternion functions
#multiply, exponential, log, conjugate, rotate vectors
import numpy as np
#import quaternion #for testing
import os
import math
import transforms3d

#allowed library functions:
#Euler2Rot, Rot2Euler, Euler2Quat, Quat2Euler, Spherical2Cartesian, Cartesian2Spherical"
#-------------------------------------------------------------
#implemented functions:
#--quaternion multiplication
#--quaternion exponential
#--quaternion log
#--quaternion conjugate
#--hat map
#--inverse hat map
#--q_rot()???
#--quaternion average
#--axis angle to quat
#--quaternion multiplication
#--euler to rot
#--rot to euler
#--quat to euler
#-------------------------------------------------------------
#need:
#--euler to quat
#--quat to rotation
#--rotation to quat
#-------------------------------------------------------------

def qmult(q, p):
    #q = 4 length vector [qs, qv] = [[qs],[i,j,k]]
    if q.size != 4 or p.size != 4:
        return -1, -1
    qs = q[0]
    ps = p[0]
    qv = q[1:4]
    pv = p[1:4]
    s = qs*ps - np.dot(qv, pv)
    v = qs*pv + ps*qv + np.cross(qv,pv)
    output = np.empty(4)
    output[0] = s
    output[1:4] = v
    return output

def q_exp(q):
    qs = q[0]
    qv = q[1:4]
    qv_mag = np.linalg.norm(qv)
    q_mag = np.linalg.norm(q)
    output = np.empty(4)
    output[0] = np.cos(qv_mag)
    if qv_mag == 0:
        output[1:4] = np.zeros(3)
    else:
        output[1:4] = qv/qv_mag*np.sin(qv_mag)
    output = output*np.exp(qs)
    return output

def q_log(q):
    qs = q[0]
    qv = q[1:4]
    if np.linalg.norm(qv) == 0:
        return np.array([np.log(np.linalg.norm(q)), 0, 0, 0])
    qv_mag = np.linalg.norm(qv)
    q_mag = np.linalg.norm(q)
    output = np.empty(4)
    output[0] = np.log(q_mag)
    output[1:4] = qv/qv_mag*np.arccos(qs/q_mag)
    return output

def q_conjugate(q):
    qs = q[0]
    qv = q[1:4]
    output = np.empty(4)
    output[0] = qs
    output[1:4] = qv*-1
    if np.linalg.norm(output) != 1:
        output = output / np.sqrt(np.dot(output, output))
    return output

def hat(q):
    q_hat = np.array([[0,-1*q[2],q[1]],[q[2],0,-1*q[0]],[-1*q[1],q[0],0]])
    return q_hat

def inv_hat(q_hat):
    #q_hat = np.array([[0,-1*q[2],q[1]],[q[2],0,-1*q[0]],[-1*q[1],q[0],0]])
    q = np.array([q_hat[2,1],q_hat[0,2],q_hat[1,0]])
    return q

def q_rot(q, x): #what is this?
    qs = q[0]
    qv = q[1:4]
    qv_mag = np.linalg.norm(qv)
    q_mag = np.linalg.norm(q)

    E_q = np.empty((3,4))
    E_q[:,0] = -1 * qv
    E_q[:,1:4] = np.eye((3,3))*qs + hat(qv)

    G_q = np.empty((3, 4))
    G_q[:, 0] = -1 * qv
    G_q[:, 1:4] = np.eye((3, 3)) * qs - hat(qv)

    R_q = np.dot(E_q,G_q.T)

    output = np.empty(4)
    output[0] = 0
    output[1:4] = np.dot(R_q,x)

    return output


def quaternion_avg(q_list, weights):
    #https://www.mathworks.com/matlabcentral/fileexchange/40098-tolgabirdal-averaging-quaternions
    A = np.zeros((4,4))
    M = len(q_list)
    wSum = 0

    for i in range(M):
        q = q_list[i]
        q = q[..., None]
        w_i = weights[i]
        A=w_i*(np.dot(q,q.T)+A)
        wSum = wSum + w_i

    A = (1.0/wSum)*A;
    eig_vals, Qavg  = np.linalg.eig(A)

    return Qavg[:,0]


def quat_average(qs, ws): 
    #q = 4 length vector [qs, qv] = [[qs],[i,j,k]]
    #second one, from class
    #input is a list of quaternions and their weights, quaternion format?
    #start = time.time()
    pi = np.pi
    out=qs[0,:] #initialize out to first quaternion
    n=qs.shape[0] #--> this will be 7
    ev_s = np.zeros((n,3))
    for j in range(100):
        for i in range(0,n):
            #start = time.time()
            inverse = transforms3d.quaternions.qinverse(out)
            qi = qmult(inverse, qs[i,:])
            ev_i = (2*q_log(qi))[1:4]
            #ev_i = ev_i[1:4]
            ev_i_norm = np.linalg.norm(ev_i)
            if ev_i_norm == 0:
                ev_i_norm = 10**-10
            ev_i = ((pi + ev_i_norm)%(2*pi)-pi ) * ev_i / ev_i_norm
            ev_s[i] = ev_i
            #end=time.time()
            #if i == 1 and j == 0:
            #    print(end-start)
            #print(temp.shape)
        #start = time.time()
        ev_average = np.sum(ev_s.T*ws, axis = 1)
        out = qmult(out, q_exp(np.asarray([0,ev_average[0],ev_average[1],ev_average[2]])/2))
        ev = np.linalg.norm(ev_average)
        if(ev<0.0001):
            break
        #end = time.time()
        #if j == 1:
        #    print(end-start)
        #    print(ev)
    
    return out 

def euler_axis_angle_to_quat(euler_angle, axis):
    q = np.empty(4)
    q[0] = np.cos(euler_angle/2)
    q[1:4] = np.sin(euler_angle/2)*axis
    return q


##fix this one
def eulerAnglesToRotationMatrix(theta):
    #from online
    R_x = np.array([[1, 0, 0],
                    [0, math.cos(theta[0]), -math.sin(theta[0])],
                    [0, math.sin(theta[0]), math.cos(theta[0])]
                    ])

    R_y = np.array([[math.cos(theta[1]), 0, math.sin(theta[1])],
                    [0, 1, 0],
                    [-math.sin(theta[1]), 0, math.cos(theta[1])]
                    ])

    R_z = np.array([[math.cos(theta[2]), -math.sin(theta[2]), 0],
                    [math.sin(theta[2]), math.cos(theta[2]), 0],
                    [0, 0, 1]
                    ])

    R = np.dot(R_z, np.dot(R_y, R_x))

    return R



def quat_to_euler(quaternion):
    #http: // graphics.wikia.com / wiki / Conversion_between_quaternions_and_Euler_angles
    q0 = quaternion[0]
    q1 = quaternion[1]
    q2 = quaternion[2]
    q3 = quaternion[3]

    roll = np.arctan(2*(q0*q1+q2*q3)/(1-2*(q1**2+q2**2)))
    pitch = np.arcsin(2*(q0*q1-q3*q1))
    yaw = np.arctan(2*(q0*q3+q1*q2)/(1-2*(q2**2+q3**2)))
    return np.array([roll,pitch,yaw])

def euler_to_quat(euler):
    r = euler[0]
    p = euler[1]
    y = euler[2]

    q = np.array([
        [np.cos(r / 2) * np.cos(p / 2) * np.cos(y / 2) + np.sin(r / 2) * np.sin(p / 2) * np.sin(y / 2)]
        [np.sin(r / 2) * np.cos(p / 2) * np.cos(y / 2) - np.cos(r / 2) * np.sin(p / 2) * np.sin(y / 2)]
        [np.cos(r / 2) * np.sin(p / 2) * np.cos(y / 2) + np.sin(r / 2) * np.cos(p / 2) * np.sin(y / 2)]
        [np.cos(r / 2) * np.cos(p / 2) * np.sin(y / 2) - np.sin(r / 2) * np.sin(p / 2) * np.cos(y / 2)]
    ])

def quat_to_rotation(q):
    q0 = q[0]
    q1 = q[1]
    q2 = q[2]
    q3 = q[3]

    return np.array([
        [1 - 2*(q2**2 + q3**2), 2*(q1*q2 - q0*q3), 2*(q0*q2 + q1*q3)],
        [2*(q1*q2 + q0*q3), 1 - 2*(q1**2 + q3**2), 2*(q2*q3 - q0*q1)],
        [2*(q1*q3 - q0*q2), 2*(q0*q1 + q2*q3), 1 - 2*(q1**2 + q2**2)]])

def rotation_to_quat(matrix):
    #found online?
    M = np.array(matrix, dtype=np.float64, copy=False)[:4, :4]
    m00 = M[0, 0]
    m01 = M[0, 1]
    m02 = M[0, 2]
    m10 = M[1, 0]
    m11 = M[1, 1]
    m12 = M[1, 2]
    m20 = M[2, 0]
    m21 = M[2, 1]
    m22 = M[2, 2]
    # symmetric matrix K
    K = np.array([[m00 - m11 - m22, 0.0, 0.0, 0.0],
                     [m01 + m10, m11 - m00 - m22, 0.0, 0.0],
                     [m02 + m20, m12 + m21, m22 - m00 - m11, 0.0],
                     [m21 - m12, m02 - m20, m10 - m01, m00 + m11 + m22]])
    K /= 3.0
    # quaternion is eigenvector of K that corresponds to largest eigenvalue
    w, V = np.linalg.eigh(K)
    q = V[[3, 0, 1, 2], np.argmax(w)]
    if q[0] < 0.0:
        np.negative(q, q)
    return q

#def r3_to_quat(r3):
'''
q1 = np.quaternion(1,2,3,4)
q2 = np.quaternion(5,6,7,8)
b1 = np.array([1,2,3,4])
b2 = np.array([5,6,7,8])

a = q1*q2
b = q_mult(b1,b2)
print('Reference multiplication: {}, my multiplication: {}'.format(a,b))

#a = exp(q1) #not working
b = q_exp(b1)
print('my exponential: {}'.format(b))

b = q_ (b1)
print('my log: {}'.format(b))

'''
'''
axis = np.array([1,0,0])
q1 = euler_axis_angle_to_quat(np.pi/2,axis)
q2 = euler_axis_angle_to_quat(np.pi/3,axis)
q3 = euler_axis_angle_to_quat(np.pi/4,axis)

b1 = euler_axis_angle_to_quat(170*np.pi/180,axis)
b2 = euler_axis_angle_to_quat(270*np.pi/180,axis)
b3 = euler_axis_angle_to_quat(-101*np.pi/180,axis)

avg = quaternion_avg([q1,q2,q3], np.ones(3))

os.chdir('/Users/dimitri/Google Drive/Documents/class_notes/grad_school/ECE276a/project2/code/transforms3d/transforms3d')
'''

