##################################################################################################################
## This is the implementation for solo impedance controlle (12 DOF).
## This code works with just pybullet and does not depend on dynamic graph.
## Primarly designed for designing and debuggin controllers
#################################################################################################################
## Author: Avadesh Meduri
## Date: 20/09/2019
#################################################################################################################


from py_blmc_controllers.impedance_controller import ImpedanceController, ImpedanceControllerSolo8
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
        self.num_eef = 0
        config_file = 'yaml_example.yaml' #Should be read in
        initialise_controllers(config_file)

    def initialise_controllers(self, config_file):
        self.imps = []
        with open(config_file) as config:
            data_in = yaml.safe_load(config)
        for ctrl_name in data_in["impedance_controllers"]:
            if int(data_in["impedance_controllers"][ctrl_name]["is_eeff"]):
                self.num_eef += 1
            #Add check if quadruped_robot has the appropriate root and end frames
            self.imps.append(ImpedanceController(ctrl_name, \
                                self.quadruped_robot.pin_robot, \
                                data_in["impedance_controllers"][ctrl_name]["frame_foot_name"], \
                                data_in["impedance_controllers"][ctrl_name]["frame_end_name"], \
                                int(data_in["impedance_controllers"][ctrl_name]["start_column"])
                                ))
            

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

    def return_joint_torques(self, q, dq, kp, kd, x_des, xd_des, f):
        '''
        Returns the joint torques at the current timestep
        '''

        tau = zero(self.num_eef*3)
        tau[0:3] = self.FL_imp.compute_impedance_torques(q,dq,kp[0:3],kd[0:3],x_des[0:3],xd_des[0:3],f[0:3])
        tau[3:6] = self.FR_imp.compute_impedance_torques(q,dq,kp[3:6],kd[3:6], x_des[3:6], xd_des[3:6],f[3:6])
        tau[6:9] = self.HL_imp.compute_impedance_torques(q,dq,kp[6:9],kd[6:9], x_des[6:9],xd_des[6:9],f[6:9])
        tau[9:12] = self.HR_imp.compute_impedance_torques(q,dq,kp[9:12],kd[9:12], x_des[9:12],xd_des[9:12], f[9:12])

        return tau

    def return_joint_torques_world(self, q, dq, kp, kd, x_des, xd_des, f):
        """Returns joint torques with x_des and xd_des in world coordinates."""
        tau = zero(12)
        for i, imp in enumerate(self.imps):
            s = slice(3*i, 3*(i+1))
            tau[s] = imp.compute_impedance_torques_world(
                q, dq, kp[s], kd[s], x_des[s], xd_des[s], f[s]
            )

        return tau

class Solo8ImpedanceController(SoloImpedanceController):
    def initialise_leg_impedance(self):
        '''
        Creates the springs behaviour between the hip and foot
        '''

        self.FL_imp = ImpedanceControllerSolo8(self.quadruped_leg_names[0] + "_imp",\
                                 self.quadruped_robot.pin_robot, \
                                 self.quadruped_leg_names[0] + self.quadruped_name_connector[0] + self.quadruped_frame_names[0],\
                                 self.quadruped_leg_names[0] + self.quadruped_name_connector[0] + self.quadruped_frame_names[1],\
                                    6)

        self.FR_imp = ImpedanceControllerSolo8(self.quadruped_leg_names[1] + "_imp",\
                                 self.quadruped_robot.pin_robot, \
                                 self.quadruped_leg_names[1] + self.quadruped_name_connector[0] + self.quadruped_frame_names[0],\
                                 self.quadruped_leg_names[1] + self.quadruped_name_connector[0] + self.quadruped_frame_names[1],\
                                     8)

        self.HL_imp = ImpedanceControllerSolo8(self.quadruped_leg_names[2] + "_imp",\
                                 self.quadruped_robot.pin_robot, \
                                 self.quadruped_leg_names[2] + self.quadruped_name_connector[0] + self.quadruped_frame_names[0],\
                                 self.quadruped_leg_names[2] + self.quadruped_name_connector[0] + self.quadruped_frame_names[1],\
                                     10)

        self.HR_imp = ImpedanceControllerSolo8(self.quadruped_leg_names[3] + "_imp",\
                                 self.quadruped_robot.pin_robot, \
                                 self.quadruped_leg_names[3] + self.quadruped_name_connector[0] + self.quadruped_frame_names[0],\
                                 self.quadruped_leg_names[3] + self.quadruped_name_connector[0] + self.quadruped_frame_names[1],\
                                     12)

        self.imps = [self.FL_imp, self.FR_imp, self.HL_imp, self.HR_imp]

    def return_joint_torques(self, q, dq, kp, kd, x_des, xd_des, f):
        """ Returns the joint torques at the current timestep. """

        tau = zero(14)
        for i, imp in enumerate(self.imps):
            s = slice(3*i, 3*(i+1))
            tau += imp.compute_impedance_torques(
                q, dq, kp[s], kd[s], x_des[s], xd_des[s], f[s]
            )

        return tau[6:]

    def return_joint_torques_world(self, q, dq, kp, kd, x_des, xd_des, f):
        """ Returns joint torques with x_des and xd_des in world coordinates. """
        tau = zero(14)
        for i, imp in enumerate(self.imps):
            s = slice(3*i, 3*(i+1))
            tau += imp.compute_impedance_torques_world(
                q, dq, kp[s], kd[s], x_des[s], xd_des[s], f[s]
            )

        return tau[6:]