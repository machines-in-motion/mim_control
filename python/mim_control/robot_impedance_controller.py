##################################################################################################################
## This file creates impedance controllers between different frames
## based on the input yaml file
#################################################################################################################
## Author: Avadesh Meduri & Paarth Shah
## Date: 9/12/2020
#################################################################################################################

import numpy as np
import yaml
from .impedance_controller import ImpedanceController


class RobotImpedanceController(ImpedanceController):
    def __init__(self, robot, config_file):
        """
        Input:
            robot : robot object returned by pinocchio wrapper
            config_file : file that describes the desired frames to create
                          springs in
        """

        self.robot = robot
        self.num_eef = 0
        self.imp_ctrl_array = []
        self.initialise_impedance_controllers(config_file)

    def initialise_impedance_controllers(self, config_file):
        """
        Reads the config file and initializes the impedance controllers

        Input:
            config_file : file that describes the desired frames to create
                          springs in
        """
        with open(config_file) as config:
            data_in = yaml.safe_load(config)
        for ctrls in data_in["impedance_controllers"]:
            if int(data_in["impedance_controllers"][ctrls]["is_eeff"]):
                self.num_eef += 1

            # TODO:
            # Check here to make sure frame names exist in pinocchio robot_model/data structure

            # Append to list of impedance controllers
            self.imp_ctrl_array.append(
                ImpedanceController(
                    ctrls,
                    self.robot.pin_robot,
                    data_in["impedance_controllers"][ctrls]["frame_root_name"],
                    data_in["impedance_controllers"][ctrls]["frame_end_name"],
                    int(
                        data_in["impedance_controllers"][ctrls]["start_column"]
                    ),
                    data_in["impedance_controllers"][ctrls]["active_joints"],
                )
            )

    def return_joint_torques(self, q, dq, kp, kd, x_des, xd_des, f):
        """
        Returns the joint torques at the current timestep

        Input:
            q : current joint positions
            dq : current joint velocities
            kp : Proportional gain
            kd : derivative gain
            x_des : desired lengths with respect to the root frame for each
                    controller (3*number_of_springs)
            xd_des : desired velocities with respect to the root frame
            f : feed forward forces
        """
        tau = np.zeros(
            np.sum(len(leg.active_joints) for leg in self.imp_ctrl_array[:])
        )
        s1 = slice(0, 0)
        for k in range(len(self.imp_ctrl_array)):
            s = slice(3 * k, 3 * (k + 1))
            s1 = slice(
                s1.stop, s1.stop + len(self.imp_ctrl_array[k].active_joints)
            )
            tau[s1] = self.imp_ctrl_array[k].compute_impedance_torques(
                q, dq, kp[s], kd[s], x_des[s], xd_des[s], f[s]
            )

        return tau

    def return_joint_torques_world(self, q, dq, kp, kd, x_des, xd_des, f):
        """
        Returns the joint torques at the current timestep, in
        world co-ordinates.

        Input:
            q : current joint positions
            dq : current joint velocities
            kp : Proportional gain
            kd : derivative gain
            x_des : desired lenghts (3*number_of_springs)
            xd_des : desired velocities
            f : feed forward forces
        """
        tau = np.zeros(len(self.imp_ctrl_array) * 3)
        for k in range(len(self.imp_ctrl_array)):
            s = slice(3 * k, 3 * (k + 1))
            tau[s] = self.imp_ctrl_array[k].compute_impedance_torques_world(
                q, dq, kp[s], kd[s], x_des[s], xd_des[s], f[s]
            )

        return tau
