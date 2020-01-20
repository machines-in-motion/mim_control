##################################################################################################################
## This is the implementation for solo impedance controlle (12 DOF).
## This code works with just pybullet and does not depend on dynamic graph.
## Primarly designed for designing and debuggin controllers
#################################################################################################################
## Author: Avadesh Meduri
## Date: 20/09/2019
#################################################################################################################


from py_blmc_controllers.impedance_controller import ImpedanceController
from pinocchio.utils import zero


class SoloImpedanceController(object):

    def __init__(self, quadruped_robot):
        '''
        Input:
            name : Name of the quadruped
            quadruped_robot : quadruped instance generated from the quadruped12wrapper
        '''
        self.name = "solo"
        self.quadruped_robot = quadruped_robot
        ## Change the names here when the names are changed in the URDF
        self.quadruped_leg_names = ['FL', 'FR', 'HL', 'HR']
        self.quadruped_frame_names = ['HFE', 'FOOT']
        self.quadruped_name_connector = ['_']
        self.initialise_leg_impedance()


    def initialise_leg_impedance(self):
        '''
        Creates the springs behaviour between the hip and foot
        '''

        self.FL_imp = ImpedanceController(self.quadruped_leg_names[0] + "_imp",\
                                 self.quadruped_robot.pin_robot, \
                                 self.quadruped_leg_names[0] + self.quadruped_name_connector[0] + self.quadruped_frame_names[0],\
                                 self.quadruped_leg_names[0] + self.quadruped_name_connector[0] + self.quadruped_frame_names[1],\
                                    6)

        self.FR_imp = ImpedanceController(self.quadruped_leg_names[1] + "_imp",\
                                 self.quadruped_robot.pin_robot, \
                                 self.quadruped_leg_names[1] + self.quadruped_name_connector[0] + self.quadruped_frame_names[0],\
                                 self.quadruped_leg_names[1] + self.quadruped_name_connector[0] + self.quadruped_frame_names[1],\
                                     9)

        self.HL_imp = ImpedanceController(self.quadruped_leg_names[2] + "_imp",\
                                 self.quadruped_robot.pin_robot, \
                                 self.quadruped_leg_names[2] + self.quadruped_name_connector[0] + self.quadruped_frame_names[0],\
                                 self.quadruped_leg_names[2] + self.quadruped_name_connector[0] + self.quadruped_frame_names[1],\
                                     12)

        self.HR_imp = ImpedanceController(self.quadruped_leg_names[3] + "_imp",\
                                 self.quadruped_robot.pin_robot, \
                                 self.quadruped_leg_names[3] + self.quadruped_name_connector[0] + self.quadruped_frame_names[0],\
                                 self.quadruped_leg_names[3] + self.quadruped_name_connector[0] + self.quadruped_frame_names[1],\
                                     15)

        self.imps = [self.FL_imp, self.FR_imp, self.HL_imp, self.HR_imp]

    def world_xdes_to_local(self, x_des_world):
        """
        Converts a x_des given in world frame to an x_des in coordinate system
        used by the impedance controller (relative distance between foot
        to endeffector)
        """

    def return_joint_torques(self, q, dq, kp, kd, x_des, xd_des, f):
        '''
        Returns the joint torques at the current timestep
        '''

        tau = zero(12)
        tau[0:3] = self.FL_imp.compute_impedance_torques(q,dq,kp[0:3],kd[0:3],x_des[0:3],xd_des[0:3],f[0:3])
        tau[3:6] = self.FR_imp.compute_impedance_torques(q,dq,kp[3:6],kd[3:6], x_des[3:6], xd_des[3:6],f[3:6])
        tau[6:9] = self.HL_imp.compute_impedance_torques(q,dq,kp[6:9],kd[6:9], x_des[6:9],xd_des[6:9],f[6:9])
        tau[9:12] = self.HR_imp.compute_impedance_torques(q,dq,kp[9:12],kd[9:12], x_des[9:12],xd_des[9:12], f[9:12])

        return tau
