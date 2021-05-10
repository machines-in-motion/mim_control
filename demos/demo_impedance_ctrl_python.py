"""demo_impedance_ctrl_python.py

This file is the centroidal controller demo for solo12 using the C++ code.

License BSD-3-Clause
Copyright (c) 2021, New York University and Max Planck Gesellschaft.

Author: Maximilien Naveau
"""

import numpy as np
import pinocchio as pin
from bullet_utils.env import BulletEnvWithGround
from robot_properties_solo.solo12wrapper import Solo12Robot, Solo12Config
from mim_control.impedance_controller import ImpedanceController


if __name__ == "__main__":

    # Create a Pybullet simulation environment
    env = BulletEnvWithGround()
    robot = Solo12Robot()
    robot = env.add_robot(robot)
    RobotConfig = Solo12Config
    pin_robot = robot.pin_robot

    # Create the control vector
    tau = np.zeros(robot.nb_dof)

    # ###################### impedance controller demo #################################
    root_names = ["base_link", "base_link", "base_link", "base_link"]
    endeff_names = ["FL_FOOT", "FR_FOOT", "HL_FOOT", "HR_FOOT"]
    start_column = [6, 9, 12, 15]
    ctrls = [
        ImpedanceController(
            endeff_names[i],
            pin_robot,
            root_names[i],
            endeff_names[i],
            start_column[i],
            [True, True, True],
        )
        for i, _ in enumerate(endeff_names)
    ]

    # Des state and gains
    z_des = -0.2
    x_des = np.array(
        [
            0.195,
            0.147,
            z_des,
            0.195,
            -0.147,
            z_des,
            -0.195,
            0.147,
            z_des,
            -0.195,
            -0.147,
            z_des,
        ]
    )
    xd_des = np.zeros(3)
    kp = [100, 100, 100]
    kd = [10, 10, 10]
    f = np.zeros(3)

    end_eff_ids = [
        pin_robot.model.getFrameId(endeff_name) for endeff_name in endeff_names
    ]

    q0 = np.array(RobotConfig.initial_configuration)
    dq0 = np.array(RobotConfig.initial_velocity)

    q0[0] = 0.0
    q0[1] = 0.0
    q0[2] = -z_des
    q0[3:7] = np.array([0, 0, 0, 1])

    robot.reset_state(q0, dq0)
    for i in range(10000):
        q, dq = robot.get_state()
        tau.fill(0)
        #
        for i, ctrl in enumerate(ctrls):
            tau_out = np.array(
                ctrl.compute_impedance_torques_world(
                    q,
                    dq,
                    kp,
                    kd,
                    x_des[3 * i : 3 * (i + 1)],
                    xd_des,
                    f,
                )
            )
            tau[start_column[i] - 6 : start_column[i] - 6 + 3] = tau_out
        #
        robot.send_joint_command(tau)
        env.step()
