import cloudpickle
import os
import numpy as np
import sympy as sp
import transforms3d as t3d

class robot_config:
    """ A class to calculate all the transforms and Jacobians
    for the UR5 arm. Also stores the mass of each of the links."""

    def __init__(self):

        self.num_joints = 4
        self.config_folder = 'mriArm_config'

        # create function dictionaries
        self._T = {} # for transformation matrices
        self._Tx = {}  # for transform calculations
        self._T_inv = {}  # for inverse transform calculations
        self._J = {}  # for Jacobian calculations

        self._M = []  # placeholder for (x,y,z) inertia matrices
        self._Mq = None  # placeholder for joint space inertia matrix function
        self._Mq_g = None  # placeholder for joint space gravity term function

        # set up our joint angle symbols
        self.q = [sp.Symbol('q%i' % ii) for ii in range(self.num_joints)]
        self.dq = [sp.Symbol('dq%i' % ii) for ii in range(self.num_joints)]
        # set up an (x,y,z) offset
        self.x = [sp.Symbol('x'), sp.Symbol('y'), sp.Symbol('z')]

        self.joint_names = ['j1', 'j2', 'j3', 'j4']

        self.linear_offsets = [0.0080, 0.0080, 0.0175]
        #static offset from origin to joint 0
        self.Torg1_static = sp.Matrix([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]])

        self.Torg1 = sp.Matrix([
            [1, 0, 0, 0],
            [0, sp.cos(self.q[0]), -sp.sin(self.q[0]), 0],
            [0, sp.sin(self.q[0]), sp.cos(self.q[0]), 0],
            [0, 0, 0, 1]])

        self.T1l2 = sp.Matrix([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, self.linear_offsets[0]],
            [0, 0, 0, 1]])

        # transform matrix from joint 1 to joint 2 reference frame
        self.T12 = sp.Matrix([
            [sp.cos(self.q[1]), 0, sp.sin(self.q[1]), 0],
            [0, 1, 0, 0],
            [-sp.sin(self.q[1]), 0, sp.cos(self.q[1]), 0],
            [0, 0, 0, 1]])

        # transform matrix from joint 1  to link 2
        self.T2l3 = sp.Matrix([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, self.linear_offsets[1]],
            [0, 0, 0, 1]]) * sp.Matrix([
            [sp.cos(-np.pi/2), -sp.sin(-np.pi/2), 0, 0],
            [sp.sin(-np.pi/2), sp.cos(-np.pi/2), 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]])

        # transform matrix from joint 2 to joint 3
        self.T23 = sp.Matrix([
            [sp.cos(self.q[2]), 0, sp.sin(self.q[2]), 0],
            [0, 1, 0, 0],
            [-sp.sin(self.q[2]), 0, sp.cos(self.q[2]), 0],
            [0, 0, 0, 1]])

        # transform matrix from joint 2 to link 3
        self.T3l4 = sp.Matrix([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, self.linear_offsets[2]],
            [0, 0, 0, 1]]) * sp.Matrix([
            [sp.cos(np.pi/2), -sp.sin(np.pi/2), 0, 0],
            [sp.sin(np.pi/2), sp.cos(np.pi/2), 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]])

        # transform matrix from joint 3 to joint 4
        self.T34 = sp.Matrix([
            [1, 0, 0, 0+0.0012],
            [0, 1, 0, 0-0.0005],
            [0, 0, 1, self.q[3]],
            [0, 0, 0, 1]])

        # transform matrix from joint 3 to link 4
        self.T4l5 = sp.Matrix([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]])


        # orientation part of the Jacobian (compensating for angular velocity)
        kz = sp.Matrix([0, 1, 0])
        self.J_orientation = [
            # [1, 0, 0],
            # [0, 1, 0],
            # [1, 0, 0],
            # [0, 0, 0],
            # ]
            self._calc_T('j1')[:3, :3] * kz,  # joint 1 orientation
            self._calc_T('j2')[:3, :3] * kz,  # joint 2 orientation
            self._calc_T('j3')[:3, :3] * kz,  # joint 3 orientation
            self._calc_T('j4')[:3, :3] * kz]  # joint 4 orientation

    def J(self, name, q, x=[0, 0, 0]):
        """ Calculates the transform for a joint or link0
        name string: name of the joint or link, or end-effector
        q np.array: joint angles
        """
        # check for function in dictionary
        if self._J.get(name, None) is None:
            print('Generating Jacobian function for %s' % name)
            self._J[name] = self._calc_J(
                name, x=x)
        parameters = tuple(q) + tuple(x)
        return np.array(self._J[name](*parameters))

    def T(self, name, q):
        ''' Calculates transform matrix, rather than point'''
        if self._T.get(name, None) is None:
            print('Generating transform function for %s' % name)
            self._T[name] = self._calc_T(
                name, lambdify = True)

        return self._T[name](*tuple(q))

    def Tx(self, name, q, x=[0, 0, 0]):
        """ Calculates the transform for a joint or link
        name string: name of the joint or link, or end-effector
        q list: set of joint angles to pass in to the T function
        x list: the [x,y,z] position of interest in "name"'s reference frame
        """
        # check for function in dictionary
        if self._Tx.get(name, None) is None:
            print('Generating transform function for %s' % name)
            self._Tx[name] = self._calc_Tx(
                name, x=x)
        parameters = tuple(q) + tuple(x)
        return self._Tx[name](*parameters)[:-1].flatten()

    def Tq(self, name, q):
        """ Returns euler angles for selected joint """
        if self._T.get(name, None) is None:
            print('Generating transform function for %s' % name)
            self._T[name] = self._calc_T(
                name, lambdify = True)


        #transform_mat = self._calc_T(name, lambdify = True)
        quat = t3d.quaternions.mat2quat(self._T[name](*tuple(q))[:3,:3]) #w, x, y, z, #t3d should be fast, the _calc_T I think is the slow part... 
        quat = np.roll(quat, -1)
        return quat

    def Te(self, name, q):
        """ Returns euler angles for selected joint """
        #if self._T.get(name, None) is None:
        #            print('Generating transform function for %s' % name)
        #            self._T[name] = self._calc_T(
        #                name, x=x)
        if self._T.get(name, None) is None:
            print('Generating transform function for %s' % name)
            self._T[name] = self._calc_T(
                name, lambdify = True)

        a, b, g = t3d.euler.mat2euler(self._T[name](*tuple(q))[:3,:3]) #w, x, y, z,
        return np.array([a,b,g])

    def forwardKinPos(self, q, x=[0, 0, 0]):
        positions = []
        for joint in self.joint_names:
            xyz = self.Tx(joint, q)
            positions.append(xyz)
        return np.asarray(positions)

    def forwardKinOrientation(self, q):
        orientations = []
        for joint in self.joint_names:
            quat = self.Tq(joint, q)
            orientations.append(quat)
        return np.asarray(orientations)

    def forwardKinHomogenous(self, q, inverse = False):
        homogenous = []
        for joint in self.joint_names:
            if not inverse:
                hom = self.T(joint, q)
            else:
                hom = self.T_inv(joint, q)
            homogenous.append(hom)
        return homogenous

    def T_inv(self, name, q, x=[0, 0, 0]):
        """ Calculates the inverse transform for a joint or link
        q list: set of joint angles to pass in to the T function
        """
        # check for function in dictionary
        if self._T_inv.get(name, None) is None:
            print('Generating inverse transform function for % s' % name)
            self._T_inv[name] = self._calc_T_inv(
                name=name, x=x)
        parameters = tuple(q) + tuple(x)
        return self._T_inv[name](*parameters)


    def _calc_J(self, name, x, lambdify=True):
        """ Uses Sympy to generate the Jacobian for a joint or link
        name string: name of the joint or link, or end-effector
        lambdify boolean: if True returns a function to calculate
                          the Jacobian. If False returns the Sympy
                          matrix
        """

        # check to see if we have our Jacobian saved in file
        if os.path.isfile('%s/%s.J' % (self.config_folder, name)):
            J = cloudpickle.load(open('%s/%s.J' %
                                 (self.config_folder, name), 'rb'))
        else:
            Tx = self._calc_Tx(name, x=x, lambdify=False)
            J = []
            # calculate derivative of (x,y,z) wrt to each joint
            for ii in range(self.num_joints):
                J.append([])
                J[ii].append(Tx[0].diff(self.q[ii]))  # dx/dq[ii]
                J[ii].append(Tx[1].diff(self.q[ii]))  # dy/dq[ii]
                J[ii].append(Tx[2].diff(self.q[ii]))  # dz/dq[ii]

            end_point = int(name.strip('link').strip('joint')) - 1 #subtracked 1 for 'j1' is 0th joint
            if end_point != 'EE':
                end_point = min(int(end_point) + 1, self.num_joints)
                # add on the orientation information up to the last joint
                for ii in range(end_point):
                    #print(self.J_orientation[ii])
                    #print(J[ii])
                    J[ii] = J[ii] + list(self.J_orientation[ii]) #added in the "list" conversion
                # fill in the rest of the joints orientation info with 0
                for ii in range(end_point, self.num_joints):
                    J[ii] = J[ii] + [0, 0, 0]

            # save to file
            cloudpickle.dump(J, open('%s/%s.J' %
                                     (self.config_folder, name), 'wb'))

        J = sp.Matrix(J).T  # correct the orientation of J
        if lambdify is False:
            return J
        return sp.lambdify(self.q + self.x, J)


    def _calc_T(self, name, lambdify=False):  # noqa C907
        """ Uses Sympy to generate the transform for a joint or link
        name string: name of the joint or link, or end-effector
        lambdify boolean: if True returns a function to calculate
                          the transform. If False returns the Sympy
                          matrix
        """

        if name == 'j1':
            T = self.Torg1_static * self.Torg1# * self.T1l2
        elif name == 'j2':
            T = self.Torg1_static * self.Torg1 * self.T1l2 * self.T12# * self.T2l3
        elif name == 'j3':
            T = self.Torg1_static * self.Torg1 * self.T1l2 * self.T12 * self.T2l3 * self.T23# * self.T3l4
        elif name == 'j4' or name == 'EE':
            T = self.Torg1_static * self.Torg1 * self.T1l2 * self.T12 * self.T2l3 * self.T23 * self.T3l4 * self.T34# * self.T4l5
        else:
        	raise Exception('Invalid transformation name: %s' % name)
        
        if lambdify is False:
            return T
        return sp.lambdify(self.q, T)

    def _calc_Tx(self, name, x, lambdify=True):
        """ Uses Sympy to transform x from the reference frame of a joint
        or link to the origin (world) coordinates.
        name string: name of the joint or link, or end-effector
        x list: the [x,y,z] position of interest in "name"'s reference frame
        lambdify boolean: if True returns a function to calculate
                          the transform. If False returns the Sympy
                          matrix
        """

        # check to see if we have our transformation saved in file
        if (os.path.isfile('%s/%s.T' % (self.config_folder, name))):
            Tx = cloudpickle.load(open('%s/%s.T' %
                                       (self.config_folder, name), 'rb'))
        else:
            T = self._calc_T(name=name)
            # transform x into world coordinates
            Tx = T * sp.Matrix(self.x + [1])

            # save to file
            cloudpickle.dump(Tx, open('%s/%s.T' %
                                      (self.config_folder, name), 'wb'))

        if lambdify is False:
            return Tx
        return sp.lambdify(self.q + self.x, Tx)

    def _calc_T_inv(self, name, x, lambdify=True):
        """ Return the inverse transform matrix, which converts from
        world coordinates into the robot's end-effector reference frame
        name string: name of the joint or link, or end-effector
        x list: the [x,y,z] position of interest in "name"'s reference frame
        lambdify boolean: if True returns a function to calculate
                          the transform. If False returns the Sympy
                          matrix
        """

        # check to see if we have our transformation saved in file
        if (os.path.isfile('%s/%s.T_inv' % (self.config_folder,
                                                name))):
            T_inv = cloudpickle.load(open('%s/%s.T_inv' %
                                          (self.config_folder, name), 'rb'))
        else:
            T = self._calc_T(name=name)
            rotation_inv = T[:3, :3].T
            translation_inv = -rotation_inv * T[:3, 3]
            T_inv = rotation_inv.row_join(translation_inv).col_join(
                sp.Matrix([[0, 0, 0, 1]]))

            # save to file
            cloudpickle.dump(T_inv, open('%s/%s.T_inv' %
                                         (self.config_folder, name), 'wb'))

        if lambdify is False:
            return T_inv
        return sp.lambdify(self.q + self.x, T_inv)