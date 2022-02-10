"""Centroidal controller demo for a given robot (solo in this case)

License BSD-3-Clause
Copyright (c) 2020, New York University and Max Planck Gesellschaft.

Author: Avadesh Meduri
"""

import argparse
import numpy as np
from mim_control.robot_impedance_controller import RobotImpedanceController
from mim_control.robot_centroidal_controller import RobotCentroidalController
from bullet_utils.env import BulletEnvWithGround
from robot_properties_solo.solo12wrapper import Solo12Robot, Solo12Config
from robot_properties_bolt.bolt_wrapper import BoltRobot, BoltConfig


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
        dc = [5, 5, 5]
        kb = [200, 200, 200]
        db = [1.0, 1.0, 1.0]
        qp_penalty_lin = 3 * [1e6]
        qp_penalty_ang = 3 * [1e6]
    elif robot_name == "bolt":
        robot = env.add_robot(BoltRobot)
        robot_config = BoltConfig()
        mu = 0.2
        kc = [0, 0, 100]
        dc = [0, 0, 10]
        kb = [100, 100, 100]
        db = [10.0, 10.0, 10.0]
        qp_penalty_lin = [1, 1, 1e6]
        qp_penalty_ang = [1e6, 1e6, 1]
    else:
        raise RuntimeError(
            "Robot name [" + str(robot_name) + "] unknown. "
            "Try 'solo' or 'bolt'"
        )

    # Initialize control
    tau = np.zeros(robot.nb_dof)

    # Reset the robot to some initial state.
    q0 = np.matrix(robot_config.initial_configuration).T
    q0[0] = 0.0
    dq0 = np.matrix(robot_config.initial_velocity).T
    robot.reset_state(q0, dq0)

    # Desired center of mass position and velocity.
    x_com = [0.0, 0.0, 0.18]
    xd_com = [0.0, 0.0, 0.0]
    # The base should be flat.
    x_ori = [0.0, 0.0, 0.0, 1.0]
    x_angvel = [0.0, 0.0, 0.0]
    # Alle end-effectors are in contact.
    cnt_array = robot.nb_ee * [1]

    # Impedance controller gains
    kp = robot.nb_ee * [0.0, 0.0, 0.0]  # Disable for now
    kd = robot.nb_ee * [0.0, 0.0, 0.0]
    x_des = robot.nb_ee * [0.0, 0.0, -q0[2].item()]  # Desired leg length
    xd_des = robot.nb_ee * [0.0, 0.0, 0.0]

    config_file = robot_config.ctrl_path
    robot_cent_ctrl = RobotCentroidalController(
        robot_config,
        mu=mu,
        kc=kc,
        dc=dc,
        kb=kb,
        db=db,
        qp_penalty_lin=qp_penalty_lin,
        qp_penalty_ang=qp_penalty_ang,
    )
    robot_leg_ctrl = RobotImpedanceController(robot, config_file)

    # Run the simulator for 100 steps
    for _ in range(4000):
        # Step the simulator.
        env.step(
            sleep=True
        )  # You can sleep here if you want to slow down the replay
        # Read the final state and forces after the stepping.
        q, dq = robot.get_state()
        # computing forces to be applied in the centroidal space
        w_com = robot_cent_ctrl.compute_com_wrench(
            q, dq, x_com, xd_com, x_ori, x_angvel
        )
        # distributing forces to the active end effectors
        F = robot_cent_ctrl.compute_force_qp(q, dq, cnt_array, w_com)
        # passing forces to the impedance controller
        tau = robot_leg_ctrl.return_joint_torques(
            q, dq, kp, kd, x_des, xd_des, F
        )
        # passing torques to the robot
        robot.send_joint_command(tau)


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
