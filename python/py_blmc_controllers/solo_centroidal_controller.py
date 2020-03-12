"""Centroidal

Author: Julian Viereck
Date: 3 Oct 2019
"""


import numpy as np
import pinocchio as pin

from py_blmc_controllers.qp_solver import quadprog_solve_qp

arr = lambda a: np.array(a).reshape(-1)
mat = lambda a: np.matrix(a).reshape((-1, 1))

class SoloCentroidalController(object):
    def __init__(self, robot, m, mu, kc, dc, kb, db, eff_ids):
        self._robot = robot
        self._m = m # Total robot mass.
        self._mu = mu # Friction coefficient
        self._kc = kc
        self._dc = dc
        self._kb = kb
        self._db = db
        self._eff_ids = eff_ids

    def compute_com_wrench(self, t, q, dq, des_pos, des_vel, des_ori, des_angvel):
        """Compute the desired COM wrench (equation 1).

        Args:
            t: Timestep to compute w_com
            des_pos: desired center of mass position at time t
            des_vel: desired center of mass velocity at time t
            des_ori: desired base orientation at time t (quaternions)
            des_angvel: desired base angular velocity at time t
        Returns:
            Computed w_com
        """
        m = self._m
        robot = self._robot

        com = arr(robot.com(q, dq)[0])
        vcom = arr(robot.vcom(q, dq))
        Ib = robot.mass(q)[3:6, 3:6]

        quat_diff = self.quaternion_difference(arr(q[3:7]), arr(des_ori))

        w_com = np.hstack([
            m * np.multiply(self._kc, des_pos - com) + m * np.multiply(self._dc, des_vel - vcom),
            arr(arr( np.multiply(self._kb, quat_diff)) + (Ib * mat(np.multiply(self._db, des_angvel - arr(dq[3:6])))).T)
        ])

        return w_com

    def compute_force_qp(self, t, q, dq, cnt_array, w_com):
        """Computes the forces needed to generated a desired centroidal wrench.
        Args:
            t: Timestep.
            q: Generalized robot position configuration.
            q: Generalized robot velocity configuration.
            cnt_array: Array with {0, 1} of #endeffector size indicating if
                an endeffector is in contact with the ground or not. Forces are
                only computed for active endeffectors.
            w_com: Desired centroidal wrench to achieve given forces.
        Returns:
            Computed forces as a plain array of size 3 * num_endeffectors.
        """

        robot = self._robot
        com = robot.com(q, dq)[0]
        foot_loc = [robot.data.oMf[i].translation for i in self._eff_ids]
        r = [robot.data.oMf[i].translation - com for i in self._eff_ids]

        # Use the contact activation from the plan to determine which of the forces
        # should be active.
        N = (int)(np.sum(cnt_array))

        # Setup the QP problem.
        Q = 2. * np.eye(3 * N + 6)
        Q[-6:,-6:] = 1e6 * np.eye(6)
        p = np.zeros(3 * N + 6)
        A = np.zeros((6, 3 * N + 6))
        b = w_com

        G = np.zeros((5 * N, 3 * N + 6))
        h = np.zeros((5 * N))

        j = 0
        for i in range(4):
            if cnt_array[i] == 0:
                continue

            A[:3, 3 * j:3 * (j + 1)] = np.eye(3)
            A[3:, 3 * j:3 * (j + 1)] = pin.utils.skew(r[i])

            G[5*j + 0, 3 * j + 0] = 1    # mu Fz - Fx >= 0
            G[5*j + 0, 3 * j + 2] = -self._mu
            G[5*j + 1, 3 * j + 0] = -1     # mu Fz + Fx >= 0
            G[5*j + 1, 3 * j + 2] = -self._mu
            G[5*j + 2, 3 * j + 1] = 1    # mu Fz - Fy >= 0
            G[5*j + 2, 3 * j + 2] = -self._mu
            G[5*j + 3, 3 * j + 1] = -1     # mu Fz + Fy >= 0
            G[5*j + 3, 3 * j + 2] = -self._mu
            G[5*j + 4, 3 * j + 2] = -1     # Fz >= 0

            j += 1

        A[:, -6:] = np.eye(6)

        solx = quadprog_solve_qp(Q, p, G, h, A, b)

        F = np.zeros(12)
        j = 0
        for i in range(4):
            if cnt_array[i] == 0:
                continue
            F[3*i: 3*(i + 1)] = solx[3*j: 3*(j + 1)]
            j += 1

        return F


    #### quaternion stuff
    def skew(self, v):
        '''converts vector v to skew symmetric matrix'''
        assert v.shape[0] == 3, 'vector dimension is not 3 in skew method'
        return np.array([[0., -v[2], v[1]],
                         [v[2], 0., -v[0]],
                         [-v[1], v[0], 0.]])
    def quaternion_to_rotation(self, q):
        ''' converts quaternion to rotation matrix '''
        return (q[3]**2 - q[:3].dot(q[:3]))*np.eye(3) \
            + 2. * np.outer(q[:3], q[:3]) + 2.*q[3]*self.skew(q[:3])

    def exp_quaternion(self, w):
        ''' converts angular velocity to quaternion '''
        qexp = np.zeros(4)
        th = np.linalg.norm(w)
        if th**2 <= 1.e-6:
            ''' small norm causes closed form to diverge,
            use taylor expansion to approximate '''
            qexp[:3] = (1-(th**2)/6)*w
            qexp[3] = 1-(th**2)/2
        else:
            u = w/th
            qexp[:3] = np.sin(th)*u
            qexp[3] = np.cos(th)
        return qexp

    def log_quaternion(self, q):
        """ lives on the tangent space of SO(3) """
        v = q[:3]
        w = q[3]
        vnorm = np.linalg.norm(v)
        if vnorm <= 1.e-6:
            return 2 * v/w * (1 - vnorm**2/(3*w**2))
        else:
            return 2*np.arctan2(vnorm, w) * v / vnorm

    def quaternion_product(self, q1, q2):
        """ computes quaternion product of q1 x q2 """
        p = np.zeros(4)
        p[:3] = np.cross(q1[:3], q2[:3]) + q2[3]*q1[:3] + q1[3]*q2[:3]
        p[3] = q1[3]*q2[3] - q1[:3].dot(q2[:3])
        return p

    def integrate_quaternion(self, q, w):
        """ updates quaternion with tangent vector w """
        dq = self.exp_quaternion(.5*w)
        return self.quaternion_product(dq, q)

    def quaternion_difference(self, q1, q2):
        """computes the tangent vector from q1 to q2 at Identity
        returns vecotr w
        s.t. q2 = exp(.5 * w)*q1
        """
        # first compute dq s.t.  q2 = q1*dq
        q1conjugate = np.array([-q1[0], -q1[1], -q1[2], q1[3]])
        # order of multiplication is very essential here
        dq = self.quaternion_product(q2, q1conjugate)
        # increment is log of dq
        return self.log_quaternion(dq)
