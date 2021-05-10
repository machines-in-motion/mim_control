"""demo_impedance_ctrl_cpp

This file is the impedance controller demo for solo12 using the C++ code.

License BSD-3-Clause
Copyright (c) 2021, New York University and Max Planck Gesellschaft.

Author: Maximilien Naveau
"""

import numpy as np

np.set_printoptions(precision=2, suppress=True)
import pinocchio
from bullet_utils.env import BulletEnvWithGround
import pybullet
from robot_properties_solo.solo12wrapper import Solo12Robot, Solo12Config
from mim_control_cpp import ImpedanceController


if __name__ == "__main__":

    # Create a Pybullet simulation environment
    env = BulletEnvWithGround()
    robot = Solo12Robot()
    robot = env.add_robot(robot)
    pybullet.resetDebugVisualizerCamera(1.3, 100, -35, (0.0, 0.0, 0.0))
    RobotConfig = Solo12Config
    pin_robot = robot.pin_robot

    # Create the control vector
    q_null = np.zeros(pin_robot.nq)
    tau = np.zeros(robot.nb_dof)
    tau_imp = np.zeros(robot.nb_dof)
    tau_pd = np.zeros(robot.nb_dof)

    # ###################### impedance controller demo ########################
    root_names = ["base_link", "base_link", "base_link", "base_link"]
    endeff_names = ["FL_FOOT", "FR_FOOT", "HL_FOOT", "HR_FOOT"]
    ctrls = [ImpedanceController() for _ in endeff_names]
    for i, ctrl in enumerate(ctrls):
        ctrl.initialize(pin_robot.model, root_names[i], endeff_names[i])

    # Des state and gains
    x_des = [
        pinocchio.SE3(np.identity(3), np.array([0.195, 0.147, -0.2])),
        pinocchio.SE3(np.identity(3), np.array([0.195, -0.147, -0.2])),
        pinocchio.SE3(np.identity(3), np.array([-0.195, 0.147, -0.2])),
        pinocchio.SE3(np.identity(3), np.array([-0.195, -0.147, -0.2])),
    ]
    xd_des = pinocchio.Motion.Zero()
    kp = np.array([200, 200, 200, 0, 0, 0])
    kd = np.array([5.0, 5.0, 5.0, 0, 0, 0])
    f = pinocchio.Force.Zero()

    end_eff_ids = [
        pin_robot.model.getFrameId(endeff_name) for endeff_name in endeff_names
    ]

    # Reset the robot to some initial state.
    q0 = np.array(RobotConfig.initial_configuration)
    dq0 = np.array(RobotConfig.initial_velocity)
    q0[0] += 0.0
    q0[1] += 0.0
    q0[2] += 0.5
    q0[3:7] = np.array([0.0, 0.0, 0.0, 1.0])
    robot.reset_state(q0, dq0)

    time = 0
    for i in range(2000):
        q, dq = robot.get_state()
        tau.fill(0)
        tau_imp.fill(0)
        tau_pd.fill(0)

        tau_pd[:] = -2.0 * q[7:] - 0.1 * dq[6:]

        for i, ctrl in enumerate(ctrls):
            ctrl.run(
                q,
                dq,
                kp,
                kd,
                0.0,
                x_des[i],
                xd_des,
                f,
            )
            tau_imp += ctrl.get_joint_torques()

        for i in range(robot.nb_dof):
            if tau_imp[i] != 0.0:
                tau[i] = tau_imp[i]
            else:
                tau[i] = tau_pd[i]

        time += 1
        robot.send_joint_command(tau)
        env.step()
