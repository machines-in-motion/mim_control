##################################################################################################################
## This file is the centroidal controller demo for solo12 using the C++ code.
#################################################################################################################
## Author: Julian
## Date: 01/21/2021
#################################################################################################################

import numpy as np
import time
import os

np.set_printoptions(precision=2, suppress=True)

import pinocchio as pin
from bullet_utils.env import BulletEnvWithGround
from robot_properties_solo.solo12wrapper import Solo12Robot, Solo12Config
import mim_control_cpp


if __name__ == "__main__":

    # Create a Pybullet simulation environment
    env = BulletEnvWithGround()
    robot = env.add_robot(Solo12Robot, useFixedBase=True)
    RobotConfig = Solo12Config
    pin_robot = robot.pin_robot

    # Create the control vector
    tau = np.zeros(robot.nb_dof)

    # ###################### impedance controller demo #################################
    root_names = ["base_link", "base_link", "base_link", "base_link"]
    endeff_names = ["FL_FOOT", "FR_FOOT", "HL_FOOT", "HR_FOOT"]
    ctrls = [mim_control_cpp.ImpedanceController3D() for _ in endeff_names]
    for i, ctrl in enumerate(ctrls):
        ctrl.initialize(pin_robot.model, root_names[i], endeff_names[i])

    # Des state and gains
    x_des = [
        0.195,
        0.147,
        -0.2,
        0.195,
        -0.147,
        -0.2,
        -0.195,
        0.147,
        -0.2,
        -0.195,
        -0.147,
        -0.2,
    ]
    xd_des = pin.Motion.Zero()
    kp = [200, 200, 200]
    kd = [10.0, 10.0, 10.0]
    f = pin.Force.Zero()

    end_eff_ids = [
        pin_robot.model.getFrameId(endeff_name) for endeff_name in endeff_names
    ]

    N = 20
    for n in range(N):
        # Reset the robot to some initial state.
        q0 = np.array(RobotConfig.initial_configuration)
        dq0 = np.array(RobotConfig.initial_velocity)

        q0[1] += 0.3
        q0[1] += 0.1
        q0[2] += 0.2
        q0[3:7] = pin.Quaternion(
            pin.rpy.rpyToMatrix(
                np.array([float(n) / float(N) * 2 * np.pi, 0, 0])
            )
        ).coeffs()

        robot.reset_state(q0, dq0)
        for i in range(1000):
            q, dq = robot.get_state()
            tau.fill(0)
            for i, ctrl in enumerate(ctrls):
                ctrl.run(
                    q,
                    dq,
                    kp,
                    kd,
                    0.0,
                    pin.SE3(np.eye(3), np.array(x_des[3 * i : 3 * (i + 1)])),
                    xd_des,
                    f,
                )
                tau += ctrl.get_joint_torques()

            robot.send_joint_command(tau)
            env.step()
