import os
import numpy as np
import sympy as sp
import transforms3d as t3d
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.axes3d import Axes3D
import cloudpickle


pi = np.pi
head_path = f"/home/{os.environ['USER']}/Documents/igr/src/software_interface/vrep_robot_control"

class dh_robot_config_kinematics:
    ''' 
    provides a robot class for forward and inverse kinematics, jacobian calculation based 
    on modified DH parameters as defined in 'Introduction to Robotics, Mechanics and Control'.

    Implemented through Sympy to generate callable function 
    '''
    
    def __init__(self, num_joints, alpha, theta, D, a, jointType, Tbase, L, M, folder_name):
        
        '''
        D-H parameter includes: number of joints, D, a, alpha, theta and 
            jointType = prismatic or revolute type of joints 
            Tbase = transformation of base frame with respect to world frame
            L = COM of each link with respect to their D-H frames.
            M = Mass and principal inertia matrix of each link that has size of 
                [m, m, m, Ixx, Iyy, Izz]
            folder_name = where to save and read the calculated Jacobian, Mass and Gravity matrix
        '''
        self.num_joints = num_joints
        self.alpha = alpha
        self.theta = theta
        self.D = D
        self.a = a
        self.jointType = jointType
        self.config_folder = folder_name
        self.num_links = num_joints + 1  # number of links include base arms
        
        # joint angles
        self.q = [sp.Symbol('q%i' % (ii+1)) for ii in range(self.num_joints)]
        # joint speed
        self.dq = [sp.Symbol('dq%i' % (ii+1)) for ii in range(self.num_joints)]
        # The position of point with respect to its own DH frames
        # default is [0, 0, 0]
        self.x = [sp.Symbol('x'), sp.Symbol('y'), sp.Symbol('z')]
             
        # Base DH-frame transformation with respect to world frame
        self._Tbase = Tbase
        self._Tj2j = []  # transformation from joint to joint
        self._Tj2l = []  # transformation from joint to link
        self._Tjoint = []  # transformation from base frame to joint
        self._Tlink = []  # transformation from base frame to link
        
        # Rotatioin matrix 
        # lambda = mathematical representation contain joint angles
        self._Rjointlambda = []
        self._Rlinklambda = []
        self._Tjointlambda = []
        self._Tlinklambda = []

        # Transformation of joint and link with respect to world frame 
        # generated callable function 
        self._Tx_joint = []
        self._Tx_link = []
        self._Tx_inv_joint = []
        self._Tx_inv_link = []

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

                # transformation between DH frames based on modified DH parameters
                t = sp.Matrix([
                    [sp.cos(theta), -sp.sin(theta), 0, a],
                    [sp.sin(theta)*sp.cos(alpha), sp.cos(theta)*sp.cos(alpha), -sp.sin(alpha), -D*sp.sin(alpha)],
                    [sp.sin(theta)*sp.sin(alpha), sp.cos(theta)*sp.sin(alpha), sp.cos(alpha), D*sp.cos(alpha)],
                    [0, 0, 0, 1]])    
                self._Tj2j.append(t)

            # transformation from DH frames to the COM of each arm
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

    def _calc_Tx(self, T, lambdify = True):
        # generate lambdified function of transformation matrix from 
        Tx =  T * sp.Matrix(self.x + [1])  # appends a 1 to the column vector x
        if lambdify:
            return sp.lambdify(self.q + self.x, Tx)
        return Tx

    def Tx(self, name, i, q, x=[0, 0, 0]):
        """ Calculate transformation matrix of a point with respect to world frame given its
            poisiton with respect to its own DH frame
            
            name : string = 'link' or 'joint' that represent the transformation of COM or DH frames
            i : int = 
            q : list = joint angles in joint space
            x : list = position of base frame with respect to world frame 
        """
        parameters = tuple(q) + tuple(x)
        if name == 'joint':
            return self._Tx_joint[i](*parameters)
        else:
            return self._Tx_joint[i](*parameters)

    def Tx_inv(self, name, i, q, x=[0, 0, 0]):
        """ Calculate Inverse transformation matrix of a point with respect to world frame given its
            poisiton with respect to its own DH frame
            
            name : string = 'link' or 'joint' that represent the transformation of COM or DH frames
            i : int = index of which joint
            q : list = joint angles in joint space
            x : list = position of base frame with respect to world frame 
        """
        parameters = tuple(q) + tuple(x)
        if name == 'joint':
            return self._Tx_inv_joint[i](*parameters)


if __name__ == '__main__':      
    param = ['D', 'a', 'alpha', 'theta', 'num_joints', 'jointType', 'Tbase', 'L', 'M']
    config = dict()
    for i in range(len(param)):
        config[param[i]] = np.load('./robot_config/inbore/config/%s.npy'%param[i])
    
    robot = dh_robot_config_kinematics(int(config['num_joints']), config['alpha'], config['theta'], config['D'], config['a'], 
                                                config['jointType'], config['Tbase'], config['L'], config['M'], 'robot_config/inbore')
    robot.initKinematicTransforms()