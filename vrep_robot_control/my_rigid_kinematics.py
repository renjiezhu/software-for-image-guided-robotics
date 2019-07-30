import os
import numpy as np
import sympy as sp
import transforms3d as t3d
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.axes3d import Axes3D
import cloudpickle

import time #for testing code speed

pi = np.pi

class dh_robot_config:
    ''' provides a robot class for forward and inverse kinematics, jacobian calculation based on modified DH parameters as defined in Introduction to Robotics, Mechanics and Control'''
    
    def __init__(self, num_joints, alpha, theta, D, a, jointType, ai, aj, ak):
        
        self.num_joints = num_joints
        self.alpha = alpha
        self.theta = theta
        self.D = D
        self.a = a
        self.jointType = jointType
        self.config_folder = 'robot_config'
        
        self.q = [sp.Symbol('q%i' % (ii+1)) for ii in range(self.num_joints)]
        self.x = [sp.Symbol('x'), sp.Symbol('y'), sp.Symbol('z')]
             
        self._Tbase = np.eye(4)
        self._Tbase[:3,:3] = t3d.euler.euler2mat(ai, aj, ak)  #construct base transform
        self._Tjoint = []  # transformation from joint to joint
        self._T = []  # transformation from base frame to joint
        self._Tlambda = []  
        self._Rlambda = []
        self._Tx = []
        self._Tx_inv = []
        self._J_position = []
        self._J_orientation = []
        
        # accounting for mass and gravity term
        self._M = []
        self._Mq = []
        self._Gq = []
        
        #constructs transform matrices for joints
        for i in range(self.num_joints):
            theta = self.theta[i]
            a = self.a[i]
            D = self.D[i]
            alpha = self.alpha[i]
            
            if self.jointType[i] == 'r':
                theta += self.q[i]
            elif self.jointType[i] == 'p':
                D += self.q[i]            
            else:
                print("error in joint {} type, neither prismatic (p) or revolute (r)".format(i))
                
            t = sp.Matrix([
                [sp.cos(theta), -sp.sin(theta), 0, a],
                [sp.sin(theta)*sp.cos(alpha), sp.cos(theta)*sp.cos(alpha), -sp.sin(alpha), -D*sp.sin(alpha)],
                [sp.sin(theta)*sp.sin(alpha), sp.cos(theta)*sp.sin(alpha), sp.cos(alpha), D*sp.cos(alpha)],
                [0, 0, 0, 1]])    
            self._Tjoint.append(t)
        
        self._M.append(np.diag([5.539, 5.539, 5.539, 2.152e-2, 7.838e-1, 2.866e-2]))
        self._M.append(np.diag([1.491e-2, 1.491e-2, 1.491e-2, 6.36e-2, 3.97e-4, 6.128e-1]))
        
        
    def initKinematicTransforms(self):
        
        #constructs transform matrices from base and inverse
        for i in range(self.num_joints):
            if i > 0:
                self._T.append(self._T[i-1]*self._Tjoint[i])
            else:
                self._T.append(self._Tbase*self._Tjoint[i]) #notice this includes the base transform
                
            self._Tlambda.append(sp.lambdify(self.q, self._T[i]))
            self._Rlambda.append(sp.lambdify(self.q, self._T[i][:3,:3]))
            
            self._Tx.append(self._calc_Tx(i)) #
            
            #calculate inverse transforms
            Tx = self._T[i]
            rotation_inv = Tx[:3, :3].T
            translation_inv = -rotation_inv * Tx[:3, 3]
            Tx_inv = rotation_inv.row_join(translation_inv).col_join(
            sp.Matrix([[0, 0, 0, 1]]))
            self._Tx_inv.append(sp.lambdify(self.q + self.x, Tx_inv))
            
            #calculate position jacobian
            self._J_position.append(self._calc_J_position(i, lambdify = True))
            self._J_orientation.append(self._calc_J_orientation(i, lambdify = True))
            
                    
    
    def _calc_Tx(self, i, lambdify = True):
        Tx = self._T[i] * sp.Matrix(self.x + [1]) #appends a 1 to the column vector x
        if lambdify:
            return sp.lambdify(self.q + self.x, Tx)
        return Tx
    
    def Tx(self, i, q, x=[0, 0, 0]):
        parameters = tuple(q) + tuple(x)
        return self._Tx[i](*parameters)[:-1].flatten()
    
    def Orientation_quaternion(self, i, q):
        parameters = tuple(q)
        return t3d.quaternions.mat2quat(self._Rlambda[i](*parameters))
    
    def Orientation_euler(self, i, q):
        parameters = tuple(q)
        return t3d.euler.mat2euler(self._Rlambda[i](*parameters))
    
    def Orientation_axAngle(self, i, q):
        parameters = tuple(q)
        return t3d.axangles.mat2axangle(self._Rlambda[i](*parameters))
    
    def _calc_J_position(self, i, lambdify = True):
        Tx = self._calc_Tx(i, lambdify = False)
        J = Tx.jacobian(sp.Matrix(self.q))[:3,:]
        if lambdify:
            return sp.lambdify(self.q + self.x, J)
        return J
    
    def J_position(self, i, q, x=[0, 0, 0]):
        parameters = tuple(q) + tuple(x)
        return np.array(self._J_position[i](*parameters))
    
    def _calc_J_orientation(self, i, lambdify = True):
        J = sp.zeros(3,i+1)
        
        for j in range(i+1):
            T = self._T[j]
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
    
    def J_orientation(self, i, q, x=[0, 0, 0]):
        parameters = tuple(q) + tuple(x)
        return np.array(self._J_orientation[i](*parameters))
    
    def _calc_T_inv(self, name, x, lambdify=True):
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
            # get the Jacobians for each link's COM
            J = [self._calc_J('link%s' % ii, x=[0, 0, 0], lambdify=False)
                 for ii in range(self.num_links)]

            # transform each inertia matrix into joint space
            # sum together the effects of arm segments' inertia on each motor
            Mq = sp.zeros(self.num_joints)
            for ii in range(self.num_links):
                Mq += J[ii].T * self._M[ii] * J[ii]
            Mq = sp.Matrix(Mq)

            # save to file
            cloudpickle.dump(Mq, open('%s/Mq' % self.config_folder, 'wb'))

        if lambdify is False:
            return Mq
        return sp.lambdify(self.q + self.x, Mq)

    def _calc_Mq_g(self, lambdify=True):
        """ Uses Sympy to generate the force of gravity in
        joint space for the ur5
        lambdify boolean: if True returns a function to calculate
                          the Jacobian. If False returns the Sympy
                          matrix
        """

        # check to see if we have our gravity term saved in file
        if os.path.isfile('%s/Mq_g' % self.config_folder):
            Mq_g = cloudpickle.load(open('%s/Mq_g' % self.config_folder,
                                         'rb'))
        else:
            # get the Jacobians for each link's COM
            J = [self._calc_J('link%s' % ii, x=[0, 0, 0], lambdify=False)
                 for ii in range(self.num_links)]

            # transform each inertia matrix into joint space and
            # sum together the effects of arm segments' inertia on each motor
            Mq_g = sp.zeros(self.num_joints, 1)
            for ii in range(self.num_joints):
                Mq_g += J[ii].T * self._M[ii] * self.gravity
            Mq_g = sp.Matrix(Mq_g)

            # save to file
            cloudpickle.dump(Mq_g, open('%s/Mq_g' % self.config_folder,
                                        'wb'))

        if lambdify is False:
            return Mq_g
        return sp.lambdify(self.q + self.x, Mq_g)


    def plot3D(self, q):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        
        plot_sphere(np.zeros(3), 0.075, ax, color = 'b')
        points = np.empty([self.num_joints, 3])
        for i in range(self.num_joints):
            points[i,:] = self.Tx(i = i, q = q)
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
    
    sp.init_printing()
    robot = dh_robot_config(num_joints = 4, alpha = [-pi/2, pi/2, 0, pi/2], theta = [0, pi/2, 0, pi/2], D = [0, 0, 0, 0], a = [0, 0, 0, 0], jointType = ['p', 'p', 'p', 'r'], ai = pi/2, aj = 0, ak = 0)
    for i in range(robot.num_joints):
        sp.pprint(robot._T[i])
        print('\n')
        
    #check FK matches up by hand
    print(robot.Tx(2, [0.5, 0, 0, 0])) #-0.5 y
    print(robot.Tx(2, [0., 0.5, 0, 0])) #0.5 z
    print(robot.Tx(2, [0., 0., 0.5, 0])) #0.5 x
    
    #check Jacobian matches up by hand
    for i in range(robot.num_joints):
        print('position jacobian:\n')
        sp.pprint(robot._calc_J_position(i, lambdify = False))
        print('orientation jacobian:\n')
        sp.pprint(robot._calc_J_orientation(i, lambdify = False))
        print('\n')
        
   #time FK and Jacobian calculations
    num_iters = 100000
    qs = np.random.rand(num_iters,4)
    
    start = time.time()
    for i in range(num_iters):
        robot.Tx(3, qs[i])
    print('Time per iter for FK over {} iters: {}'.format(num_iters, (time.time()-start)/num_iters))

    
    start = time.time()
    for i in range(num_iters):
        robot.J_position(3, qs[i])
        robot.J_orientation(3, qs[i])
    print('Time per iter for Jacobian over {} iters: {}'.format(num_iters, (time.time()-start)/num_iters))
    
    
    print('Orientation quaternion test: {}'.format(robot.Orientation_quaternion(3,qs[-1])))
          
     