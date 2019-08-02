import os
import numpy as np
import sympy as sp
import transforms3d as t3d
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.axes3d import Axes3D
from config import *
import cloudpickle
import time #for testing code speed

pi = np.pi


class dh_robot_config:
    ''' provides a robot class for forward and inverse kinematics, jacobian calculation based 
    on modified DH parameters as defined in Introduction to Robotics, Mechanics and Control.

    The Mass and Inertia matrix, Gravity matrix are also included with respect to 
    '''
    
    def __init__(self, num_joints, alpha, theta, D, a, jointType, Tbase):
        
        '''D-H parameter includes: D, a, alpha, theta
        ai, aj, ak are the euler transformation of base with respect to the world frame
        x = [x, y, z] is the initial of base frame in world frame
        '''
        self.num_joints = num_joints
        self.alpha = alpha
        self.theta = theta
        self.D = D
        self.a = a
        self.jointType = jointType
        self.config_folder = 'robot_config'
        self.num_links = num_joints + 1
        
        self.q = [sp.Symbol('q%i' % (ii+1)) for ii in range(self.num_joints)]
        self.dq = [sp.Symbol('dq%i' % (ii+1)) for ii in range(self.num_joints)]
        self.x = [sp.Symbol('x'), sp.Symbol('y'), sp.Symbol('z')]
             
        self._Tbase = Tbase
        self._Tj2j = []  # transformation from joint to joint
        self._Tj2l = []  # transformation from joint to link
        self._Tjoint = []  # transformation from base frame to joint
        self._Tlink = []  # transformation from base frame to link
        
        self._Rjointlambda = []
        self._Rlinklambda = []
        self._Tjointlambda = []
        self._Tlinklambda = []
        self._Tx_joint = []
        self._Tx_link = []
        self._Tx_inv_joint = []
        self._Tx_inv_link = []
        self._J_position_joint = []
        self._J_orientation_joint = []
        self._J_position_link = []
        self._J_orientation_link = []
        self._J_joint = []
        self._J_link = []
        
        # accounting for mass and gravity term
        self._M = []
        self._Mq = []
        self._Gq = []
        self.gravity = sp.Matrix([0,0,-9.81,0,0,0])

        self._L = L
        #constructs transform matrices for joints
        for i in range(self.num_joints + 1):
            self._M.append(np.diag(M[i]))
            
            if i>0 :
                theta = self.theta[i-1]
                a = self.a[i-1]
                D = self.D[i-1]
                alpha = self.alpha[i-1]
            
                if self.jointType[i-1] == 'r':
                    theta += self.q[i-1]
                elif self.jointType[i-1] == 'p':
                    D += self.q[i-1]            
                else:
                    print("error in joint {} type, neither prismatic (p) or revolute (r)".format(i-1))
                
                t = sp.Matrix([
                    [sp.cos(theta), -sp.sin(theta), 0, a],
                    [sp.sin(theta)*sp.cos(alpha), sp.cos(theta)*sp.cos(alpha), -sp.sin(alpha), -D*sp.sin(alpha)],
                    [sp.sin(theta)*sp.sin(alpha), sp.cos(theta)*sp.sin(alpha), sp.cos(alpha), D*sp.cos(alpha)],
                    [0, 0, 0, 1]])    
                self._Tj2j.append(t)

            l = sp.Matrix([
                [1, 0, 0, self._L[i, 0]],
                [0, 1, 0, self._L[i, 1]],
                [0, 0, 1, self._L[i, 2]],
                [0, 0, 0, 1]
            ])
            self._Tj2l.append(l)


    def initKinematicTransforms(self):
        #constructs transform matrices from base to joint and link
        for i in range(self.num_joints + 1):
            # Transformation from world base to joint 
            if i < self.num_joints:
                if i > 0:
                    self._Tjoint.append(self._Tjoint[i-1]*self._Tj2j[i])
                else:
                    self._Tjoint.append(self._Tbase*self._Tj2j[i])  # notice this includes the base transform
            # Transformation from world base to link 
            if i > 0:
                self._Tlink.append(self._Tjoint[i-1]*self._Tj2l[i])
            else:
                self._Tlink.append(self._Tbase*self._Tj2l[i])
            '''
            # Rotation matrix of joint and link that can be calledb
            if i < self.num_joints:
                self._Rjointlambda.append(sp.lambdify(self.q, self._Tjoint[i][:3,:3]))
            self._Rlinklambda.append(sp.lambdify(self.q, self._Tlink[i][:3,:3]))
            # Transformation of joint and link with respect to world frame
            # Default is [0, 0, 0] (Robot base frame coincide with world frame)
            if i < self.num_joints:
                self._Tx_joint.append(self._calc_Tx(self._Tjoint[i]))
            self._Tx_link.append(self._calc_Tx(self._Tlink[i]))
            # calculate inverse transformations of joint and link
            if i < self.num_joints:
                Tx = self._Tjoint[i]
                rotation_inv = Tx[:3, :3].T
                translation_inv = -rotation_inv * Tx[:3, 3]
                Tx_inv = rotation_inv.row_join(translation_inv).col_join(sp.Matrix([[0, 0, 0, 1]]))
                self._Tx_inv_joint.append(sp.lambdify(self.q + self.x, Tx_inv))
            Tx = self._Tlink[i]
            rotation_inv = Tx[:3, :3].T
            translation_inv = -rotation_inv * Tx[:3, 3]
            Tx_inv = rotation_inv.row_join(translation_inv).col_join(sp.Matrix([[0, 0, 0, 1]]))
            self._Tx_inv_link.append(sp.lambdify(self.q + self.x, Tx_inv))
            # calculate jacobian of joint
            if i < self.num_joints:
                self._J_position_joint.append(self._calc_J_position(self._Tjoint[i], lambdify = True))
                self._J_orientation_joint.append(self._calc_J_orientation(self._Tjoint[i], i, lambdify = True))
                self._J_joint.append(self._calc_J(self._Tjoint[i], i, lambdify = True))
            # calculate jacobian of link
            self._J_position_link.append(self._calc_J_position(self._Tlink[i], lambdify = True))
            self._J_orientation_link.append(self._calc_J_orientation(self._Tlink[i], i, lambdify = True))
            self._J_link.append(self._calc_J(self._Tlink[i], i, lambdify = True))
            print(len(self._J_link))
            # calculate mass and gravity matrix in joint space
        self._Mq.append(self._calc_Mq(lambdify=True))
        self._Gq.append(self._calc_Gq(lambdify=True))
        '''
    def _calc_Tx(self, T, lambdify = True):
        Tx =  T * sp.Matrix(self.x + [1])  # appends a 1 to the column vector x
        if lambdify:
            return sp.lambdify(self.q + self.x, Tx)
        return Tx

    def Tx(self, T, q, x=[0, 0, 0]):  # --------------------------------------------- need to be revised
        parameters = tuple(q) + tuple(x)
        return T(*parameters)[:-1].flatten()
    
    def Orientation_quaternion(self, i, q):  # --------------------------------------------- need to be revised
        parameters = tuple(q)
        return t3d.quaternions.mat2quat(self._Rjointlambda[i](*parameters))
    
    def Orientation_euler(self, i, q):  # --------------------------------------------- need to be revised
        parameters = tuple(q)
        return t3d.euler.mat2euler(self._Rjointlambda[i](*parameters))
    
    def Orientation_axAngle(self, i, q):  # --------------------------------------------- need to be revised
        parameters = tuple(q)
        return t3d.axangles.mat2axangle(self._Rjointlambda[i](*parameters))
    
    def _calc_J_position(self, T, lambdify = True):
        Tx = self._calc_Tx(T, lambdify = False)
        J = Tx.jacobian(sp.Matrix(self.q))[:3,:]
        if lambdify:
            return sp.lambdify(self.q + self.x, J)
        return J
    
    def _calc_J_orientation(self, T, x, lambdify = True):
        i=x if x==0 else x-1
        J = sp.zeros(3,self.num_joints)
        for j in range(i+1):
            if self.jointType[j] == 'r':
                z_hat = sp.Matrix([0, 0, 1])
            elif self.jointType[j] == 'p':
                z_hat = sp.Matrix([0, 0, 0])
            else:
                print('joint type is not revolute (r) or prismatic (p)')
                break
            J[:,j] = T[:3,:3] * z_hat
        if lambdify:
            return sp.lambdify(self.q + self.x, J)
        return J
    
    def _calc_J(self, T, i, lambdify=True):
        x = self._calc_J_position(T, lambdify=False)
        y = self._calc_J_orientation(T, i, lambdify=False)
        if lambdify:
                sp.lambdify(self.q + self.x, x.col_join(y))
        return x.col_join(y)
        
    def Jx(self, T, q, x=[0, 0, 0]):
        parameters = tuple(q) + tuple(x)
        return np.array(T(*parameters))

    def Tx_inv(self, name, x, lambdify=True):  # --------------------------------------------- need to be revised
        pass
    
    def _calc_Mq(self, lambdify=True):
        """ Uses Sympy to generate the inertia matrix in
        joint space for the ur5
        lambdify boolean: if True returns a function to calculate
                          the Jacobian. If False returns the Sympy
                          matrix
        """

        # check to see if we have our inertia matrix saved in file
        if os.path.isfile('%s/Mq' % self.config_folder):
            Mq = cloudpickle.load(open('%s/Mq' % self.config_folder, 'rb'))
        else:
            # transform each inertia matrix into joint space
            # sum together the effects of arm segments' inertia on each motor
            Mq = sp.zeros(self.num_joints)
            for ii in range(self.num_links):
                Mq += self._J_link[ii].T * self._M[ii] * self._J_link[ii]
            Mq = sp.Matrix(Mq)
            # save to file
            # cloudpickle.dump(Mq, open('%s/Mq' % self.config_folder, 'wb'))
        if lambdify is False:
            return Mq
        return sp.lambdify(self.q + self.x, Mq)

    def _calc_Gq(self, lambdify=True):
        """ Uses Sympy to generate the force of gravity in
        joint space for the ur5
        lambdify boolean: if True returns a function to calculate
                          the Jacobian. If False returns the Sympy
                          matrix
        """
        # check to see if we have our gravity term saved in file
        if os.path.isfile('%s/Gq' % self.config_folder):
            Gq = cloudpickle.load(open('%s/Gq' % self.config_folder,
                                         'rb'))
        else:
            # transform each inertia matrix into joint space and
            # sum together the effects of arm segments' inertia on each motor
            Gq = sp.zeros(self.num_joints, 1)
            for ii in range(self.num_joints):
                Gq += self._J_link[ii].T * self._M[ii] * self.gravity
            Gq = sp.Matrix(Gq)
            # save to file
            # cloudpickle.dump(Gq, open('%s/Gq' % self.config_folder,
            #                            'wb'))
        if lambdify is False:
            return Gq
        return sp.lambdify(self.q + self.x, Gq)


    def plot3D(self, q):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        
        plot_sphere(np.zeros(3), 0.075, ax, color = 'b')
        points = np.empty([self.num_joints, 3])
        for i in range(self.num_joints):
            points[i,:] = self.Tx(self._Tx_joint[i], q = q)
            if self.jointType[i] == 'r':
                lineColor = 'b-'
            else:
                lineColor = 'r-'
                
            if i == 0:
                x = np.hstack([0, points[i,0]])
                y = np.hstack([0, points[i,1]])
                z = np.hstack([0, points[i,2]])
            else:
                x = points[i-1:i+1,0]
                y = points[i-1:i+1,1]
                z = points[i-1:i+1,2]
            
            ax.plot(x, y, z, lineColor,linewidth=5)
            plot_sphere(points[i,:], 0.05, ax)
        
        print(points)
        plt.show()

        
def plot_sphere_data(position, radius):
    """Given a position and radius, get the data needed to plot.
    :param position: Position in (x, y, z) of sphere
    :type position: numpy.ndarray
    :param radius: radius of sphere
    :type radius: int
    :returns: (x, y, z) tuple of sphere data to use to create a surface
    :rtype: (np.ndarray, np.ndarray, np.ndarray)
    """
    u = np.linspace(0, 2 * np.pi, 100)
    v = np.linspace(0, np.pi, 100)

    x = (radius * np.outer(np.cos(u), np.sin(v)) +
         position[0])
    y = (radius * np.outer(np.sin(u), np.sin(v)) +
         position[1])
    z = (radius * np.outer(np.ones(np.size(u)), np.cos(v)) +
         position[2])

    return (x, y, z)
                
def plot_sphere(position, radius, ax, color='g', linewidth=0):
    """Plot a sphere.
    :param position: Position in (x, y, z) of sphere
    :type position: numpy.ndarray
    :param radius: radius of sphere
    :type radius: int
    :param ax: axes to plot on
    :type ax: matplotlib.axes
    :param color: (Optional) color of sphere
    :type color: str
    :param linewidth: (Optional) width of ball gridlines
    :type linewidth: int
    :rtype: matplotlib.axes
    """
    x, y, z = plot_sphere_data(position, radius)
    return ax.plot_surface(x, y, z, rstride=4, cstride=4,
color=color, linewidth=0)       
    
if __name__ == '__main__':
    ct_robot = dh_robot_config(num_joints, alpha, theta, D, a, jointType, ai, aj, ak)
    ct_robot.initKinematicTransforms()
    # print(ct_robot._J_joint)
    # print(ct_robot._J_link)