"""Demo of the impedance controllers between different frames based on the
input yaml file

License BSD-3-Clause
Copyright (c) 2020, New York University and Max Planck Gesellschaft.

Author: Avadesh Meduri & Paarth Shah
"""

import argparse
import numpy as np
from mim_control.robot_impedance_controller import RobotImpedanceController
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
    elif robot_name == "bolt":
        robot = BoltRobot()
        robot = env.add_robot(robot)
        robot_config = BoltConfig()
    else:
        raise RuntimeError(
            "Robot name [" + str(robot_name) + "] unknown. "
            "Try 'solo' or 'bolt'"
        )

    # Initialize control
    tau = np.zeros(robot.nb_dof)

    # Reset the robot to some initial state.
    q0 = np.matrix(robot_config.initial_configuration).T
    dq0 = np.matrix(robot_config.initial_velocity).T
    robot.reset_state(q0, dq0)

    # Impedance controller gains
    kp = robot.nb_ee * [200, 200, 200]
    kd = robot.nb_ee * [10.0, 10.0, 10.0]

    # Desired leg length.
    x_des = robot.nb_ee * [0.0, 0.0, -q0[2].item()]
    xd_des = robot.nb_ee * [0.0, 0.0, 0.0]

    # distributing forces to the active end-effectors
    f = np.zeros(robot.nb_ee * 3)
    f = robot.nb_ee * [0.0, 0.0, (robot_config.mass * 9.8) / 4]

    config_file = robot_config.ctrl_path
    robot_leg_ctrl = RobotImpedanceController(robot, config_file)

    # Run the simulator for 100 steps
    for _ in range(4000):
        # Step the simulator.
        env.step(
            sleep=True
        )  # You can sleep here if you want to slow down the replay
        # Read the final state and forces after the stepping.
        q, dq = robot.get_state()
        # passing forces to the impedance controller
        tau = robot_leg_ctrl.return_joint_torques(
            q, dq, kp, kd, x_des, xd_des, f
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
