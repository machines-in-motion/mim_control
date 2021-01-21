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

from mim_control.robot_impedance_controller import RobotImpedanceController
from bullet_utils.env import BulletEnvWithGround
from robot_properties_solo.solo12wrapper import Solo12Robot, Solo12Config

import mim_control_cpp

# Create a Pybullet simulation environment
env = BulletEnvWithGround()

robot = env.add_robot(Solo12Robot)
RobotConfig = Solo12Config

tau = np.zeros(robot.nb_dof)

# # Reset the robot to some initial state.

q0 = np.matrix(RobotConfig.initial_configuration).T
dq0 = np.matrix(RobotConfig.initial_velocity).T
q0[0] = 0.

robot.reset_state(q0, dq0)

# ###################### impedance controller demo #################################

x_des = robot.nb_ee*[0.0, 0.0, -0.25]
xd_des = robot.nb_ee*[0,0,0]
kp = robot.nb_ee * [200,200,200]
kd = robot.nb_ee * [10.0,10.0,10.0]
f = np.zeros(robot.nb_ee*3)
f = robot.nb_ee*[0.0, 0.0, (2.2*9.8)/4]

root_name = 'universe'
endeff_names = ['FL_ANKLE', 'FR_ANKLE', 'HL_ANKLE', 'HR_ANKLE']
ctrls = [mim_control_cpp.ImpedanceController() for eff_name in endeff_names]
for i, c in enumerate(ctrls):
    c.initialize(robot.pin_robot.model, root_name, endeff_names[i])

x_des = [
  0.195,  0.147, 0.015,
  0.195, -0.147, 0.015,
 -0.195,  0.147, 0.015,
 -0.195, -0.147, 0.015
]

q_init = np.zeros(19)
q_init[7] = 1

robot_leg_ctrl = RobotImpedanceController(robot, Solo12Config.paths['imp_ctrl_yaml'])

centrl_pd_ctrl = mim_control_cpp.CentroidalPDController()
centrl_pd_ctrl.initialize(2.5, np.diag(robot.pin_robot.mass(q_init)[3:6, 3:6]))

force_qp = mim_control_cpp.CentroidalForceQPController()
force_qp.initialize(4, 0.2, 5e5, 1e6)

robot.reset_state(q0, dq0)
pin_robot = robot.pin_robot

end_eff_ids = []
for leg in ["FL", "FR", "HL", "HR"]:
    end_eff_ids.append(pin_robot.model.getFrameId(leg + "_FOOT"))

for i in range(5000):
    q, dq = robot.get_state()

    w_com = np.zeros(6)
    w_com[2] += 9.81 * 2.5

    x_com = [0.0, 0.0, 0.18]
    xd_com = [0.0, 0.0, 0.0]
    x_ori = [0., 0., 0., 1.]
    x_angvel = [0., 0., 0.]
    cnt_array = [1, 1, 1, 1]

    centrl_pd_ctrl.run(
        [200., 200., 200.,], [50., 50., 50.], [100., 100., 200.], [50., 50., 200.],
        q[:3], x_com, dq[:3], xd_com,
        q[3:7], x_ori, dq[3:6], x_angvel
    )

    w_com += centrl_pd_ctrl.get_wrench()

    # distrubuting forces to the active end effectors
    pin_robot = robot.pin_robot
    pin_robot.framesForwardKinematics(q)
    com = pin_robot.com(q)
    rel_eff = np.array([
        pin_robot.data.oMf[i].translation - com for i in end_eff_ids
    ]).reshape(-1)
    force_qp.run(w_com, rel_eff, cnt_array)
    F = force_qp.get_forces()

    tau = np.zeros(18)
    for i, c in enumerate(ctrls):
        c.run(q, dq,
                 np.array([200,200,200,0,0,0]),
                 np.array([10,10,10,0,0,0]),
                 1.,
                 pin.SE3(np.eye(3), np.array(x_des[3*i:3*(i+1)])),
                 pin.Motion(np.zeros(3), np.zeros(3)),
                 pin.Force(np.array(F[3*i:3*(i+1)]), np.zeros(3))
             )
        tau += c.get_torques()

    robot.send_joint_command(tau[6:])
    env.step()