##################################################################################################################
## This is the implementation for bolt impedance controlle.
## This code works with just pybullet and does not depend on dynamic graph.
## Primarly designed for designing and debuggin controllers
#################################################################################################################

from py_blmc_controllers.impedance_controller import ImpedanceController
from pinocchio.utils import zero


class BoltImpedanceController(object):

    def __init__(self, bolt_robot):
        '''
        Input:
            name : Name of the bolt
            bolt_robot : bolt instance generated from the bolt_wrapper
        '''
        self.name = "bolt"
        self.bolt_robot = bolt_robot
        ## Change the names here when the names are changed in the URDF
        self.bolt_leg_names = ['FL', 'FR']
        self.bolt_frame_names = ['HFE', 'FOOT']
        self.bolt_name_connector = ['_']
        self.initialise_leg_impedance()


    def initialise_leg_impedance(self):
        '''
        Creates the springs behaviour between the hip and foot
        '''

        self.FL_imp = ImpedanceController(self.bolt_leg_names[0] + "_imp",\
                                 self.bolt_robot.pin_robot, \
                                 self.bolt_leg_names[0] + self.bolt_name_connector[0] + self.bolt_frame_names[0],\
                                 self.bolt_leg_names[0] + self.bolt_name_connector[0] + self.bolt_frame_names[1],\
                                    6)

        self.FR_imp = ImpedanceController(self.bolt_leg_names[1] + "_imp",\
                                 self.bolt_robot.pin_robot, \
                                 self.bolt_leg_names[1] + self.bolt_name_connector[0] + self.bolt_frame_names[0],\
                                 self.bolt_leg_names[1] + self.bolt_name_connector[0] + self.bolt_frame_names[1],\
                                     9)

        self.imps = [self.FL_imp, self.FR_imp]

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

        tau = zero(6)
        r = zero(6)
        tau[0:3] = self.FL_imp.compute_impedance_torques(q,dq,kp[0:3],kd[0:3],x_des[0:3],xd_des[0:3],f[0:3])
        tau[3:6] = self.FR_imp.compute_impedance_torques(q,dq,kp[3:6],kd[3:6], x_des[3:6], xd_des[3:6],f[3:6])

        return tau
