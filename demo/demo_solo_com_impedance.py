# demo for com impedance control with 12dof solo
import time

import numpy as np
import os
import os.path
import rospkg
import pybullet as p

import eigenpy
import pinocchio as se3
from pinocchio.utils import se3ToXYZQUAT

from robot_properties_solo.config import Solo12Config
from robot_properties_solo.quadruped12wrapper import Quadruped12Robot

from py_blmc_controllers.solo_centroidal_controller import SoloCentroidalController
from py_blmc_controllers.solo_impedance_controller import SoloImpedanceController

from pinocchio.utils import zero
from matplotlib import pyplot as plt


if __name__ == "__main__":
     # Create a robot instance. This initializes the simulator as well.
    robot = Quadruped12Robot()
    tau = np.zeros(12)

    # Reset the robot to some initial state.
    q0 = np.matrix(Solo12Config.initial_configuration).T
    dq0 = np.matrix(Solo12Config.initial_velocity).T
    robot.reset_state(q0, dq0)

    arr = lambda a: np.array(a).reshape(-1) 
    mat = lambda a: np.matrix(a).reshape((-1, 1))
    total_mass = sum([i.mass for i in robot.pin_robot.model.inertias[1:]])

    x_des = 4*[0.0, 0.0, -0.25]
    xd_des = 4*[0. ,0., 0.]  
    
    kp = 4 * [20,20,50]
    kd = 4 * [1.0,1.0,1.0]
    # kp = 4 * [0 , 0, 0]
    # kd = 4 * [0.0, 0.0, 0.0]
    f = np.zeros(18)
    f = 4*[0.0, 0.0, (2.2*9.8)/4]
        
    x_com = [0.0, 0.0, 0.25]
    xd_com = [0.0, 0.0, 0.0]

    x_ori = [0., 0., 0., 1.]
    x_angvel = [0., 0., 0.]
    cnt_array = [1, 1, 1, 1]
    kp_com = 4 * [200,200,200]
    kd_com = 4 * [10.0,10.0,10.0]
    

    solo_leg_ctrl = SoloImpedanceController(robot)
    centr_controller = SoloCentroidalController(robot.pin_robot, total_mass, mu=0.6, kc=1000., dc=10., kb=50., db=0.01, 
                                                eff_ids=robot.pinocchio_endeff_ids)

    robot.reset_state(q0, dq0)
    p.stepSimulation()

    for t in range(5000):
        q, dq = robot.get_state_update_pinocchio()

        w_com = centr_controller.compute_com_wrench(t, q, dq, x_com, xd_com, x_ori, x_angvel)
        #w_com[2] += total_mass * 9.81
        F = centr_controller.compute_force_qp(t, q, dq, cnt_array, w_com)

        # if t % 100 == 0:
        #     print(w_com, F)

        tau = solo_leg_ctrl.return_joint_torques(q, dq, kp, kd, x_des, xd_des, F)
        robot.send_joint_command(tau)

        p.stepSimulation()
        time.sleep(0.001)