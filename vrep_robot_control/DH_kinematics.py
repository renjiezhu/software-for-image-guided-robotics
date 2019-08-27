import os
import numpy as np
import sympy as sp
import transforms3d as t3d
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.axes3d import Axes3D
import cloudpickle


pi = np.pi
head_path = f"/home/{os.environ['USER']}/Documents/igr/src/software_interface/vrep_robot_control"

class dh_robot_config:
    ''' 
    provides a robot class for forward and inverse kinematics, jacobian calculation based 
    on modified DH parameters as defined in 'Introduction to Robotics, Mechanics and Control'.
    '''
    
    def __init__(self, num_joints, alpha, theta, D, a, jointType, Tbase, L):
        
        '''
        D-H parameter includes: number of joints, D, a, alpha, theta and 
            jointType = prismatic or revolute type of joints 
            Tbase = transformation of base frame with respect to world frame
            L = COM of each link with respect to their D-H frames.
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
        self._J_jointlambda = []
        self._J_linklambda = []
        self._Mq = []
        self._Gq = []
        self._Mqlambda = []
        self._Gqlambda = []
        
        # accounting for mass and gravity term
        self._M = []
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
                # print('Calculating joint %d' % (i+1))
                # print('-------------------------------------------------------------------')
                self._J_position_joint.append(self._calc_J_position(self._Tjoint[i], lambdify = True))
                self._J_orientation_joint.append(self._calc_J_orientation(self._Tjoint[i], i, lambdify = True))
                self._J_joint.append(self._calc_J(self._Tjoint[i], i, 'joint', lambdify = True))
                self._J_jointlambda.append(self._calc_J(self._Tjoint[i], i, 'joint', lambdify = False))
            print('Calculating link %d' % i)
            # calculate jacobian of link
            self._J_position_link.append(self._calc_J_position(self._Tlink[i], lambdify = True))
            self._J_orientation_link.append(self._calc_J_orientation(self._Tlink[i], i, lambdify = True))
            self._J_link.append(self._calc_J(self._Tlink[i], i, 'link', lambdify = True))
            self._J_linklambda.append(self._calc_J(self._Tlink[i], i, 'link', lambdify = False))

        
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
    
    def _calc_J(self, T, i, T_type, lambdify=True):
        if os.path.isfile(head_path+'/%s/J/J_%s%d' % (self.config_folder, T_type, i)):
            J = cloudpickle.load(open(head_path+'/%s/J/J_%s%d' % (self.config_folder, T_type, i), 'rb'))
        else:
            x = self._calc_J_position(T, lambdify=False)
            y = self._calc_J_orientation(T, i, lambdify=False)
            J = x.col_join(y)
            cloudpickle.dump(J, open(head_path+'/%s/J/J_%s%d' % (self.config_folder, T_type, i), 'wb'))
        if lambdify:
            return sp.lambdify(self.q + self.x, J)
        return J
        
    def Jx(self, T, q, x=[0, 0, 0]):
        parameters = tuple(q) + tuple(x)
        return np.array(T(*parameters))

    def Tx_inv(self, name, x, lambdify=True):  # --------------------------------------------- need to be revised
        pass




if __name__ == '__main__':      
    param = ['D', 'a', 'alpha', 'theta', 'num_joints', 'jointType', 'Tbase', 'L', 'M']
    config = dict()
    for i in range(len(param)):
        config[param[i]] = np.load('./robot_config/config1/%s.npy'%param[i])
    
    robot = dh_robot_config(int(config['num_joints']), config['alpha'], config['theta'], config['D'], config['a'], 
                                                config['jointType'], config['Tbase'], config['L'], config['M'])
    robot.initKinematicTransforms()