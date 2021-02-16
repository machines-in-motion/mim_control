"""This file is a demo for using the DG whole body controller.

License BSD-3-Clause
Copyright (c) 2021, New York University and Max Planck Gesellschaft.

Author: Julian Viereck
Date:   Feb 16, 2021
"""

from dg_tools.utils import *
import dynamic_graph as dg
import dynamic_graph.sot.dynamic_pinocchio as dp

import mim_control.dynamic_graph.wbc as mim_control_dg

class WholeBodyController:
    def __init__(self, prefix, pin_robot, endeff_names, friction_coeff, qp_penalty_lin, qp_penalty_ang):
        self.pin_robot = pin_robot
        self.nv = pin_robot.model.nv
        self.ne = len(endeff_names)

        self.dg_robot = dp.DynamicPinocchio(prefix + '_pinocchio')
        self.dg_robot.setModel(self.pin_robot.model)
        self.dg_robot.setData(self.pin_robot.data)

        self.w_com_ff_sin, self.w_com_ff_sout = constVectorOp(np.zeros(6))

        ###
        # CentroidalPDController
        self.wcom_pd_ctrl = mim_control_dg.CentroidalPDController(prefix + '_wcom_pd_controller')

        q_init = np.zeros(pin_robot.nq)
        q_init[7] = 1

        self.wcom_pd_ctrl.initialize(
            np.sum([i.mass for i in pin_robot.model.inertias]),
            np.diag(pin_robot.mass(q_init)[3:6, 3:6]))

        ###
        # CentroidalForceQPController
        self.f_ctrl = mim_control_dg.CentroidalForceQPController(prefix + '_f_controller')
        self.f_ctrl.initialize(len(endeff_names), friction_coeff, qp_penalty_lin, qp_penalty_ang)

        self.endeff_names = endeff_names
        for endeff_name in self.endeff_names:
            self.dg_robot.createPosition('pos_' + endeff_name, endeff_name)

        # Create the relative leg positions.
        # TODO: Handle feet offset here as well.
        self.com_signal = self.dg_robot.signal('com')

        self.rel_eff_pos = subtract_vec_vec(
            hom2pos(self.dg_robot.signal('pos_' + self.endeff_names[0])),
            self.com_signal)

        for i in range(1, len(self.endeff_names)):
            self.rel_eff_pos = stack_two_vectors(
                self.rel_eff_pos,
                subtract_vec_vec(
                    hom2pos(self.dg_robot.signal('pos_' + self.endeff_names[i])),
                    self.com_signal
                ),
                3 * i, 3
            )

        dg.plug(add_vec_vec(
                self.wcom_pd_ctrl.wrench_sout,
                self.w_com_ff_sout
            ), self.f_ctrl.w_com_sin)
        dg.plug(self.rel_eff_pos, self.f_ctrl.relative_position_endeff_sin)


        ###
        # Impedance controllers.
        self.imps = []

        # Create plugable vector signals to control the impedance controller.
        for i, endeff_name in enumerate(self.endeff_names):
            imp = mim_control_dg.ImpedanceController(prefix + '_imp_' + endeff_name)
            imp.initialize(pin_robot.model, 'universe', endeff_name)

            dg.plug(
                stack_two_vectors(
                    selec_vector(self.f_ctrl.forces_sout, 3*i, 3*(i + 1)),
                    zero_vec(3, ''),
                    3, 3
                ),
                imp.feed_forward_force_sin)

            self.imps.append(imp)

        # The final computed control.
        self.joint_torques_sout = self.imps[0].joint_torque_sout;
        for i in range(1, self.ne):
            self.joint_torques_sout = add_vec_vec(
                    self.joint_torques_sout, self.imps[i].joint_torque_sout)

        ###
        # Export all the signals for the user of the PyEntity.
        self.kc_sin = self.wcom_pd_ctrl.kp_com_sin
        self.dc_sin = self.wcom_pd_ctrl.kd_com_sin
        self.kb_sin = self.wcom_pd_ctrl.kp_base_sin
        self.db_sin = self.wcom_pd_ctrl.kd_base_sin

        self.des_com_pos_sin = self.wcom_pd_ctrl.desired_com_position_sin
        self.des_com_vel_sin = self.wcom_pd_ctrl.desired_com_velocity_sin
        self.des_ori_pos_sin = self.wcom_pd_ctrl.desired_base_orientation_sin
        self.des_ori_vel_sin = self.wcom_pd_ctrl.desired_base_angular_velocity_sin

        self.cnt_array_sin = self.f_ctrl.cnt_array_sin


    def plug(self, robot, base_position, base_velocity):
        # Args
        #   robot; DGM robot device
        #   base_position: The base position as a 7 dim vector signal
        #   base_velocity: The base velocity as a 6 dim vector signal

        # Create the input to the dg_robot.
        base_pose_rpy = basePoseQuat2PoseRPY(base_position)
        base_velocity = base_velocity

        position = stack_two_vectors(base_pose_rpy, robot.device.joint_positions, 6, self.nv - 6)
        velocity = stack_two_vectors(base_velocity, robot.device.joint_velocities, 6, self.nv- 6)

        dg.plug(position, self.dg_robot.signal('position'))
        dg.plug(velocity, self.dg_robot.signal('velocity'))
        self.dg_robot.signal('acceleration').value = np.array(self.nv * [0.,])

        # Create the input to the impedance controllers.
        position = stack_two_vectors(base_position, robot.device.joint_positions, 7, self.nv - 6)
        for imp in self.imps:
            dg.plug(position, imp.robot_configuration_sin)
            dg.plug(velocity, imp.robot_velocity_sin)

        # Setting the actual quantities for the centroidal-pd controller.
        # NOTE: Using the base as approximation for the com.
        dg.plug(selec_vector(base_position, 0, 3), self.wcom_pd_ctrl.actual_com_position_sin)
        dg.plug(selec_vector(base_position, 3, 7), self.wcom_pd_ctrl.actual_base_orientation_sin)
        dg.plug(selec_vector(base_velocity, 0, 3), self.wcom_pd_ctrl.actual_com_velocity_sin)
        dg.plug(selec_vector(base_velocity, 3, 6), self.wcom_pd_ctrl.actual_base_angular_velocity_sin)

        # Finally, plug the computed torques to the output.
        dg.plug(self.joint_torques_sout, robot.device.ctrl_joint_torques)
