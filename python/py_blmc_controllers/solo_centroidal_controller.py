"""Centroidal

Author: Julian Viereck
Date: 3 Oct 2019
"""

from cvxopt import matrix, solvers
import numpy as np
import pinocchio as pin


class SoloCentroidalController(object):
    def __init__(self, m, mu, kc, dc, kb, db, kc_leg, kd_leg, robot, hip_ids, eff_ids):
        self._m = m # Total robot mass.
        self._mu = mu # Friction coefficient
        self._kc = kc
        self._dc = dc
        self._kb = kb
        self._db = db
        self._kc_leg = kc_leg
        self._kd_leg = kd_leg

        self._robot = robot
        self._hip_ids = hip_ids
        self._eff_ids = eff_ids

    def compute_com_wrench(self, t, q, dq, plan):
        """Compute the desired COM wrench (equation 1).

        Args:
          t: Timestep to compute w_com at
          plan: Dict with (propery, values) mapping from the planned motion
          robot: A pinocchio robot wrapper to query quantities from.
        Returns:
          Computed w_com
        """
        m = self._m
        robot = self._robot
        com = arr(robot.com(q, dq)[0])
        vcom = arr(robot.vcom(q, dq))
        Ib = robot.mass(q)[:3, :3]

        quat_diff = arr(pin.difference(robot.model, q, mat(plan['q'][t]))[3:6])

        w_com = np.hstack([
            plan['centroidal_forces'][t],
            plan['centroidal_moments'][t]
        ])

        w_com += np.hstack([
            m * self._kc * (plan['com'][t] - com) + m * self._dc * (plan['vcom'][t] - vcom),
            arr(Ib * self._kb * mat(quat_diff)) + self._db * (plan['base_ang_velocities'][t] - arr(dq[3:6]))
        ])

        return w_com

    def compute_force_qp(self, t, q, dq, plan, w_com):
        robot = self._robot
        com = robot.com(q, dq)[0]
        r = [robot.data.oMf[i].translation - com for i in self._eff_ids]

        # Use the contact activation from the plan to determine which of the forces
        # should be active.
        N = (int)(np.sum(plan['contact_activation'][t]))

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
            if plan['contact_activation'][t][i] == 0:
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

        sol = solvers.qp(matrix(Q), matrix(p), matrix(G), matrix(h), matrix(A), matrix(b),
                         options={'show_progress': False})

        solx = np.array(sol['x']).reshape(-1)

        F = np.zeros(12)
        j = 0
        for i in range(4):
            if plan['contact_activation'][t][i] == 0:
                continue
            F[3*i: 3*(i + 1)] = solx[3*j: 3*(j + 1)]

        return F
