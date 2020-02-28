##################################################################################################################
## This is the implementation for impedance controller between any two frames of the robot.
## This code works with just pybullet and does not depend on dynamic graph.
## Primarly designed for designing and debuggin controllers
#################################################################################################################
## Author: Avadesh Meduri
## Date: 20/09/2019
#################################################################################################################


import numpy as np
import pinocchio as pin
from pinocchio.utils import zero,eye


class ImpedanceController(object):

    def __init__(self, name, pin_robot, frame_root_name, frame_end_name, start_column):

        '''
        Input :
            name : Name of the impdeance controller (Ex. Front left leg impedance)
            pinocchio_robot : pinocchio wrapper instance.
            frame_root_name : The root frame name where the spring starts(Ex. Hip)
            frame_end_name : the second frame name where the spring ends(Ex. end effector)
            start_column : the column from where 3 columns from the jacobian are selected
        '''

        self.name = name
        self.pin_robot = pin_robot
        self.frame_root_name = frame_root_name
        self.frame_end_name = frame_end_name
        self.frame_root_idx = self.pin_robot.model.getFrameId(self.frame_root_name)
        self.frame_end_idx = self.pin_robot.model.getFrameId(self.frame_end_name)
        self.start_column = start_column

    def compute_forward_kinematics(self,q):
        '''
            Computes forward kinematics of all the frames and stores in data
        '''
        pin.framesForwardKinematics(self.pin_robot.model, self.pin_robot.data, q)



    def compute_distance_between_frames(self,q):
        '''
            Computes the distance between the two frames or computes the location
            of frame_end with respect to frame_root
        '''
        return self.pin_robot.data.oMf[self.frame_end_idx].translation - self.pin_robot.data.oMf[self.frame_root_idx].translation

    def compute_relative_velocity_between_frames(self,q,dq):
        '''
            computes the velocity of the end_frame with respect to a frame
            whose origin aligns with the root frame but is oriented as the world frame
        '''

        # TODO: define relative vel with respect to frame oriented as the base frame but located at root frame
        ## will be a problem in case of a back flip with current implementation. 

        frame_config_root = pin.SE3(self.pin_robot.data.oMf[self.frame_root_idx].rotation, np.zeros((3,1)))
        frame_config_end = pin.SE3(self.pin_robot.data.oMf[self.frame_end_idx].rotation, np.zeros((3,1)))

        vel_root_in_world_frame = (frame_config_root.action * pin.frameJacobian(self.pin_robot.model, self.pin_robot.data, q, self.frame_root_idx)).dot(dq)[0:3]
        vel_end_in_world_frame = (frame_config_end.action * pin.frameJacobian(self.pin_robot.model, self.pin_robot.data, q, self.frame_end_idx)).dot(dq)[0:3]

        return vel_end_in_world_frame - vel_root_in_world_frame

    def compute_jacobian(self,q):
        '''
            computes the jacobian in the world frame
            Math : J = R(World,Foot) * J_(Foot frame)
            Selection of the required portion of the jacobian is also done here
        '''
        self.compute_forward_kinematics(q)
        jac = pin.frameJacobian(self.pin_robot.model, self.pin_robot.data, q, self.frame_end_idx)
        jac = jac[:,self.start_column:self.start_column+3][0:3]
        jac = self.pin_robot.data.oMf[self.frame_end_idx].rotation.dot(jac)
        return jac

    def compute_impedance_torques(self, q, dq, kp, kd, x_des, xd_des, f):
        '''
            Computes the desired joint torques tau = -Jt * (F + kp(x-x_des) + kd(xd-xd_des))
            Inputs:
                q = joint angles
                dq = joint velocites
                Kp = proportional gain
                Kd = derivative gain
                x_des = desired [x,y,z] at time t (in the root joint frame)
                xd_des = desired velocity of end effector at time t (in the root joint frame)

        '''
        assert (np.shape(x_des) == (3,))
        assert (np.shape(xd_des) == (3,))
        assert (np.shape(f) == (3,))
        assert (np.shape(kp) == (3,))
        assert (np.shape(kd) == (3,))

        #### Reshaping values to desired shapes

        x_des = np.matrix(x_des).T
        xd_des = np.matrix(xd_des).T
        f = np.matrix(f).T
        kp = np.matrix([[kp[0],0,0],
                        [0,kp[1],0],
                        [0,0,kp[2]]])

        kd = np.matrix([[kd[0],0,0],
                        [0,kd[1],0],
                        [0,0,kd[2]]])

        #######################################

        self.compute_forward_kinematics(q)
        x = self.compute_distance_between_frames(q)
        xd = self.compute_relative_velocity_between_frames(q,dq)

        jac = self.compute_jacobian(q)
        tau = -1*jac.T.dot(f + kp*(x - x_des) + kd*(xd - xd_des))

        return tau






