##################################################################################################################
## This is the implementation for impedance controller between any two frames of the robot.
## This code works with just pybullet and does not depend on dynamic graph.
## Primarily designed for designing and debuggin controllers
#################################################################################################################
## Author: Avadesh Meduri
## Date: 20/09/2019
#################################################################################################################


import numpy as np
import pinocchio as pin
from pinocchio.utils import zero, eye


class ImpedanceController(object):
    def __init__(
        self,
        name,
        pin_robot,
        frame_root_name,
        frame_end_name,
        start_column,
        active_joints,
    ):

        """
        Input :
            name : Name of the impedance controller (Ex. Front left leg impedance)
            pinocchio_robot : pinocchio wrapper instance.
            frame_root_name : The root frame name where the spring starts(Ex. Hip)
            frame_end_name : the second frame name where the spring ends(Ex. end effector)
            start_column : the column from where 3 columns from the jacobian are selected
        """

        self.name = name
        self.pin_robot = pin_robot
        self.frame_root_name = frame_root_name
        self.frame_end_name = frame_end_name
        self.frame_root_idx = self.pin_robot.model.getFrameId(
            self.frame_root_name
        )
        self.frame_end_idx = self.pin_robot.model.getFrameId(
            self.frame_end_name
        )
        self.start_column = start_column
        self.active_joints = active_joints

    def compute_forward_kinematics(self, q):
        """
        Computes forward kinematics of all the frames and stores in data
        """
        pin.framesForwardKinematics(
            self.pin_robot.model, self.pin_robot.data, q
        )

    def compute_distance_between_frames(self, q):
        """
        Computes the distance between the two frames or computes the location
        of frame_end with respect to frame_root
        """
        return (
            self.pin_robot.data.oMf[self.frame_end_idx].translation
            - self.pin_robot.data.oMf[self.frame_root_idx].translation
        )

    def compute_relative_velocity_between_frames(self, q, dq):
        """
        computes the velocity of the end_frame with respect to a frame
        whose origin aligns with the root frame but is oriented as the world frame
        """
        # TODO: define relative vel with respect to frame oriented as the base frame but located at root frame
        ## will be a problem in case of a back flip with current implementation.

        frame_config_root = pin.SE3(
            self.pin_robot.data.oMf[self.frame_root_idx].rotation,
            np.zeros((3, 1)),
        )
        frame_config_end = pin.SE3(
            self.pin_robot.data.oMf[self.frame_end_idx].rotation,
            np.zeros((3, 1)),
        )

        vel_root_in_world_frame = frame_config_root.action.dot(
            pin.computeFrameJacobian(
                self.pin_robot.model,
                self.pin_robot.data,
                q,
                self.frame_root_idx,
            )
        ).dot(dq)[0:3]
        vel_end_in_world_frame = frame_config_end.action.dot(
            pin.computeFrameJacobian(
                self.pin_robot.model,
                self.pin_robot.data,
                q,
                self.frame_end_idx,
            )
        ).dot(dq)[0:3]

        return np.subtract(vel_end_in_world_frame, vel_root_in_world_frame).T

    def compute_jacobian(self, q):
        """
        computes the jacobian in the world frame
        Math : J = R(World,Foot) * J_(Foot frame)
        Selection of the required portion of the jacobian is also done here
        """
        self.compute_forward_kinematics(q)
        jac = pin.computeFrameJacobian(
            self.pin_robot.model, self.pin_robot.data, q, self.frame_end_idx
        )
        jac = self.pin_robot.data.oMf[self.frame_end_idx].rotation.dot(
            jac[0:3]
        )
        return jac

    def compute_impedance_torques(self, q, dq, kp, kd, x_des, xd_des, f):
        """
        Computes the desired joint torques tau = -Jt * (F + kp(x-x_des) + kd(xd-xd_des))
        Inputs:
            q = joint angles
            dq = joint velocities
            Kp = proportional gain
            Kd = derivative gain
            x_des = desired [x,y,z] at time t (in the root joint frame)
            xd_des = desired velocity of end effector at time t (in the root joint frame)

        """
        assert np.shape(x_des) == (3,)
        assert np.shape(xd_des) == (3,)
        assert np.shape(f) == (3,)
        assert np.shape(kp) == (3,)
        assert np.shape(kd) == (3,)

        #### Reshaping values to desired shapes

        x_des = np.array(x_des)
        xd_des = np.array(xd_des)
        f = np.array(f)
        kp = np.array([[kp[0], 0, 0], [0, kp[1], 0], [0, 0, kp[2]]])

        kd = np.array([[kd[0], 0, 0], [0, kd[1], 0], [0, 0, kd[2]]])

        #######################################

        self.compute_forward_kinematics(q)
        x = self.compute_distance_between_frames(q)
        xd = self.compute_relative_velocity_between_frames(q, dq)
        jac = self.compute_jacobian(q)[
            :,
            self.start_column : self.start_column + len(self.active_joints),
        ]
        jac = jac[:, self.active_joints]

        # Store force for learning project.
        self.F_ = (
            f + np.matmul(kp, (x - x_des)) + np.matmul(kd, (xd - xd_des).T).T
        )
        tau = -jac.T.dot(self.F_.T)
        final_tau = []
        j = 0
        for i in range(len(self.active_joints)):
            if self.active_joints[i] == False:
                final_tau.append(0)
            else:
                final_tau.append(tau[j])
                j += 1
        return final_tau

    def compute_impedance_torques_world(self, q, dq, kp, kd, x_des, xd_des, f):
        """Computes the leg impedance using world coordinate x_des and xd_des.

        Args:
            q: pinocchio generalized coordinates of robot
            dq: pinocchio generalized velocity of robot
            kp: (list size 3) P gains for position error.
            kd: (list size 3) D gains for velocity error.
            x_des: (list size 3) desired endeffector position size 3.
            xd_des: (list size 3) desired endeffector velocity size 3.
            f: (list size 3) feedforward force to apply at endeffector.
        """
        assert np.shape(x_des) == (3,)
        assert np.shape(xd_des) == (3,)
        assert np.shape(f) == (3,)
        assert np.shape(kp) == (3,)
        assert np.shape(kd) == (3,)

        #### Reshaping values to desired shapes

        x_des = np.array(x_des)
        xd_des = np.array(xd_des)
        f = np.array(f)
        kp = np.array(kp)
        kd = np.array(kd)

        #######################################

        self.compute_forward_kinematics(q)
        jac = self.compute_jacobian(q)

        x = self.pin_robot.data.oMf[self.frame_end_idx].translation
        xd = jac.dot(dq)

        jac = jac[
            :,
            self.start_column : self.start_column + len(self.active_joints),
        ]
        jac = jac[:, self.active_joints]

        # Store force for learning project.
        self.F_ = f + kp * (x - x_des) + kd * (xd - xd_des)
        tau = -jac.T.dot(self.F_)
        final_tau = []
        j = 0
        for i in range(len(self.active_joints)):
            if self.active_joints[i] == False:
                final_tau.append(0)
            else:
                final_tau.append(tau[j])
                j += 1
        return final_tau


class ImpedanceControllerSolo8(ImpedanceController):
    def compute_impedance_torques(self, q, dq, kp, kd, x_des, xd_des, f):
        """
        Computes the desired joint torques tau = -Jt * (F + kp(x-x_des) + kd(xd-xd_des))
        Inputs:
            q = joint angles
            dq = joint velocities
            Kp = proportional gain
            Kd = derivative gain
            x_des = desired [x,y,z] at time t (in the root joint frame)
            xd_des = desired velocity of end effector at time t (in the root joint frame)

        """
        assert np.shape(x_des) == (3,)
        assert np.shape(xd_des) == (3,)
        assert np.shape(f) == (3,)
        assert np.shape(kp) == (3,)
        assert np.shape(kd) == (3,)

        #### Reshaping values to desired shapes

        x_des = np.array(x_des)
        xd_des = np.array(xd_des)
        f = np.array(f)
        kp = np.array(kp)
        kd = np.array(kd)

        #######################################

        self.compute_forward_kinematics(q)
        x = self.compute_distance_between_frames(q)
        xd = self.compute_relative_velocity_between_frames(q, dq)

        jac = self.compute_jacobian(q)

        # Store force for learning project.
        self.F_ = f + kp * (x - x_des) + kd * (xd - xd_des)
        tau = -jac.T.dot(self.F_)
        return tau

    def compute_impedance_torques_world(self, q, dq, kp, kd, x_des, xd_des, f):
        """Computes the leg impedance using world coordinate x_des and xd_des.

        Args:
            q: pinocchio generalized coordinates of robot
            dq: pinocchio generalized velocity of robot
            kp: (list size 3) P gains for position error.
            kd: (list size 3) D gains for velocity error.
            x_des: (list size 3) desired endeffector position size 3.
            xd_des: (list size 3) desired endeffector velocity size 3.
            f: (list size 3) feedforward force to apply at endeffector.
        """
        assert np.shape(x_des) == (3,)
        assert np.shape(xd_des) == (3,)
        assert np.shape(f) == (3,)
        assert np.shape(kp) == (3,)
        assert np.shape(kd) == (3,)

        #### Reshaping values to desired shapes

        x_des = np.array(x_des)
        xd_des = np.array(xd_des)
        f = np.array(f)
        kp = np.array(kp)
        kd = np.array(kd)

        #######################################

        self.compute_forward_kinematics(q)
        jac = self.compute_jacobian(q)

        x = self.pin_robot.data.oMf[self.frame_end_idx].translation
        xd = jac.dot(dq)

        # Store force for learning project.
        self.F_ = f + kp * (x - x_des) + kd * (xd - xd_des)
        tau = -jac.T.dot(self.F_)

        return tau
