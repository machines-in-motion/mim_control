"""This file is the centroidal controller demo for solo12 using the C++ code.

License BSD-3-Clause
Copyright (c) 2020, New York University and Max Planck Gesellschaft.

Author: Julian Viereck
"""

import argparse
import numpy as np
import pinocchio
from mim_control_cpp import (
    ImpedanceController,
    CentroidalPDController,
    CentroidalForceQPController,
)
from bullet_utils.env import BulletEnvWithGround
from robot_properties_solo.solo12wrapper import Solo12Robot, Solo12Config
# from robot_properties_bolt.bolt_wrapper import BoltRobot, BoltConfig

import time

def demo(robot_name):

    # Create a Pybullet simulation environment
    env = BulletEnvWithGround()

    # Create a robot instance in the simulator.
    if robot_name == "solo":
        robot = Solo12Robot()
        robot = env.add_robot(robot)
        robot_config = Solo12Config()
        mu = 0.2
        kc = [200, 200, 200]
        dc = [50, 50, 50]
        kb = [100, 100, 200]
        db = [50.0, 50.0, 200.0]
        qp_penalty_weights = [5e5, 5e5, 5e5, 1e6, 1e6, 1e6]
    elif robot_name == "bolt":
        robot = BoltRobot()
        robot = env.add_robot(robot)
        robot_config = BoltConfig()
        mu = 0.2
        kc = [0, 0, 100]
        dc = [0, 0, 10]
        kb = [100, 100, 100]
        db = [10.0, 10.0, 10.0]
        qp_penalty_weights = [1, 1, 1e6, 1e6, 1e6, 1]
    else:
        raise RuntimeError(
            "Robot name [" + str(robot_name) + "] unknown. "
            "Try 'solo' or 'bolt'"
        )

    # alias
    pin_robot = robot.pin_robot

    # Create impedance controllers
    root_name = "universe"
    endeff_names = robot_config.end_effector_names
    end_eff_ids = [
        pin_robot.model.getFrameId(ee_name) for ee_name in endeff_names
    ]
    imp_ctrls = [ImpedanceController() for _ in endeff_names]
    for i, c in enumerate(imp_ctrls):
        c.initialize(robot.pin_robot.model, root_name, endeff_names[i])

    # create centroidal controller
    q_init = np.zeros(pin_robot.nq)
    q_init[7] = 1
    centrl_pd_ctrl = CentroidalPDController()
    centrl_pd_ctrl.initialize(
        2.5, np.diag(robot.pin_robot.mass(q_init)[3:6, 3:6])
    )
    force_qp = CentroidalForceQPController()
    force_qp.initialize(robot.nb_ee, mu, qp_penalty_weights)

    # Reset the robot to some initial state.
    q0 = np.matrix(robot_config.initial_configuration).T
    q0[0] = 0.0
    dq0 = np.matrix(robot_config.initial_velocity).T
    robot.reset_state(q0, dq0)

    # Desired center of mass position and velocity.
    x_com = [0.0, 0.0, 0.25]
    xd_com = [0.0, 0.0, 0.0]
    # The base should be flat.
    x_ori = [0.0, 0.0, 0.0, 1.0]
    x_angvel = [0.0, 0.0, 0.0]
    # Alle end-effectors are in contact.
    cnt_array = robot.nb_ee * [1]

    # impedance gains
    kp = np.array([200, 200, 200, 0, 0, 0])
    kd = np.array([10.0, 10.0, 10.0, 0, 0, 0])

    # Desired leg length
    x_des = [
        0.195,
        0.147,
        0.015,
        0.195,
        -0.147,
        0.015,
        -0.195,
        0.147,
        0.015,
        -0.195,
        -0.147,
        0.015,
    ]
    xd_des = pinocchio.Motion(np.zeros(3), np.zeros(3))

    # Run the simulator for N steps
    N = 4000
    dur = 0.
    for _ in range(N):

        # Read the final state and forces after the stepping.
        q, dq = robot.get_state()

        quat = pinocchio.Quaternion(q[6], q[3], q[4], q[5])
        quat.normalize()

        start = time.time()

        # computing forces to be applied in the centroidal space
        centrl_pd_ctrl.run(
            kc,
            dc,
            kb,
            db,
            q[:3],
            x_com,
            quat.toRotationMatrix().dot(dq[:3]), # local to world frame
            xd_com,
            q[3:7],
            x_ori,
            dq[3:6],
            x_angvel,
        )

        w_com = np.zeros(6)
        w_com[2] += 9.81 * 2.5
        w_com += centrl_pd_ctrl.get_wrench()

        # distributing forces to the active end effectors
        pin_robot.framesForwardKinematics(q)
        com = pin_robot.com(q)
        rel_eff = np.array(
            [pin_robot.data.oMf[i].translation - com for i in end_eff_ids]
        ).reshape(-1)
        force_qp.run(w_com, rel_eff, cnt_array)
        ee_forces = force_qp.get_forces()

        # passing forces to the impedance controller
        tau = np.zeros(robot.nb_dof)
        for i, imp_ctrl in enumerate(imp_ctrls):
            imp_ctrl.run(
                q,
                dq,
                kp,
                kd,
                1.0,
                pinocchio.SE3(np.eye(3), np.array(x_des[3 * i : 3 * (i + 1)])),
                pinocchio.Motion(xd_des),
                pinocchio.Force(
                    np.array(ee_forces[3 * i : 3 * (i + 1)]), np.zeros(3)
                ),
            )
            tau += imp_ctrl.get_joint_torques()

        dur += time.time() - start

        # passing torques to the robot
        robot.send_joint_command(tau)
        # Step the simulator.
        env.step(
            sleep=False
        )  # You can sleep here if you want to slow down the replay

    print('Control path: %0.3f ms' % (dur * 1000. / N))


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--solo", help="Demonstrate Solo.", action="store_true"
    )
    parser.add_argument(
        "--bolt", help="Demonstrate Bolt.", action="store_true"
    )
    args = parser.parse_args()
    if args.solo:
        robot_name = "solo"
    elif args.bolt:
        robot_name = "bolt"
    else:
        robot_name = "solo"

    demo(robot_name)
