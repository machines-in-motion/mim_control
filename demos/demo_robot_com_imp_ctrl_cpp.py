"""This file is the centroidal controller demo for solo12 using the C++ code.

License BSD-3-Clause
Copyright (c) 2020, New York University and Max Planck Gesellschaft.

Author: Julian Viereck
"""

import argparse
import numpy as np
import pinocchio
from mim_control_cpp import (
    CentroidalImpedanceController
)
from bullet_utils.env import BulletEnvWithGround
from robot_properties_solo.solo12wrapper import Solo12Robot, Solo12Config

import time

def demo(robot_name):
    # Create a Pybullet simulation environment
    env = BulletEnvWithGround()

    # Create a robot instance in the simulator.
    robot = Solo12Robot()
    robot = env.add_robot(robot)
    robot_config = Solo12Config()
    mu = 0.2
    kc = np.array([200, 200, 200])
    dc = np.array([50, 50, 50])
    kb = np.array([100, 100, 200])
    db = np.array([50.0, 50.0, 200.0])
    qp_penalty_weights = np.array([5e5, 5e5, 5e5, 1e6, 1e6, 1e6])

    # impedance gains
    kp = np.array([200, 200, 200, 0, 0, 0])
    kd = np.array([10.0, 10.0, 10.0, 0, 0, 0])

    pin_robot = robot.pin_robot

    q_init = robot_config.q0.copy()
    q_init[0] = 0.

    robot.reset_state(q_init, robot_config.v0)

    ctrl = CentroidalImpedanceController()
    ctrl.initialize(
        2.5,
        np.diag(robot.pin_robot.mass(q_init)[3:6, 3:6]),
        pin_robot.model,
        "universe",
        robot_config.end_effector_names,
        mu,
        qp_penalty_weights,
        kc, dc, kb, db,
        kp, kd
    )

    # Desired center of mass position and velocity.
    x_com = [0.0, 0.0, 0.25]
    xd_com = [0.0, 0.0, 0.0]
    # The base should be flat.
    x_ori = [0.0, 0.0, 0.0, 1.0]
    x_angvel = [0.0, 0.0, 0.0]

    # Desired leg length
    x_des = [
         0.195,  0.147, 0.015, 0, 0, 0, 1.,
         0.195, -0.147, 0.015, 0, 0, 0, 1.,
        -0.195,  0.147, 0.015, 0, 0, 0, 1.,
        -0.195, -0.147, 0.015, 0, 0, 0, 1.
    ]
    xd_des = np.zeros(4 * 6)

    dur = 0.

    # Run the simulator for N steps
    N = 4000
    for _ in range(N):
        # Read the final state and forces after the stepping.
        q, dq = robot.get_state()

        quat = pinocchio.Quaternion(q[6], q[3], q[4], q[5])
        quat.normalize()

        start = time.time()
        ctrl.run(
            q, dq,
            np.array([1., 1., 1., 1.]),
            q[:3],
            x_com,
            quat.toRotationMatrix().dot(dq[:3]), # local to world frame
            xd_com,
            q[3:7],
            x_ori,
            dq[3:6],
            x_angvel,
            x_des, xd_des
        )

        tau = ctrl.get_joint_torques()
        dur += time.time() - start

        # passing torques to the robot
        robot.send_joint_command(tau)

        # Step the simulator.
        env.step(
            sleep=False
        )  # You can sleep here if you want to slow down the replay

    print('Control path: %0.3f ms' % (dur * 1000. / N))

if __name__ == "__main__":
    demo('solo12')
