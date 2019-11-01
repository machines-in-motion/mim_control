
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

from momentumopt.utilities.motion_planner import MotionPlanner
from quadruped.quadruped_wrapper import Quadruped12Wrapper

from pinocchio.utils import zero
from matplotlib import pyplot as plt


# Generate the plan if files do not exist.
def generate_plan():
    motion_planner = MotionPlanner('cfg_quadruped_dance_solo12.yaml',
                                    RobotWrapper=Quadruped12Wrapper, with_lqr=False)
    motion_planner.optimize_motion(plot_com_motion=False)
    motion_planner.save_files()
    motion_planner.save_qp_files()
    motion_planner.replay_kinematics()


if __name__ == "__main__":
    # Check if the plan files exist. If not, generate them.
    if not os.path.exists('quadruped_com.dat'):
        print('Generating motion plan for quadruped...')
        generate_plan()


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

    read_data = lambda filename, skip=1: np.genfromtxt(filename)[:, skip:]

    plan = {
        'q': read_data('quadruped_generalized_positions.dat'),
        'dq': read_data('quadruped_generalized_velocities.dat'),
        'com': read_data('quadruped_com.dat'),
        'vcom': read_data('quadruped_com_vel.dat'),
        'centroidal_forces': read_data('quadruped_centroidal_forces.dat'),
        'centroidal_moments': read_data('quadruped_centroidal_moments.dat'),
        'base_ang_velocities': read_data('quadruped_base_ang_velocities.dat'),

        'forces': read_data('quadruped_forces.dat').reshape(-1, 4, 3),
        'contact_activation': read_data('quadruped_contact_activation.dat'),

        'eff_positions': read_data('quadruped_positions_eff.dat', skip=0),
        'eff_velocities': read_data('quadruped_velocities_eff.dat', skip=0),
    }


    q0 = mat(plan['q'][0])
    dq0 = mat(plan['dq'][0])

    kp = 4 * [200,200,200]
    kd = 4 * [10.0,10.0,10.0]

    solo_leg_ctrl = SoloImpedanceController(robot)
    centr_controller = SoloCentroidalController(robot.pin_robot, total_mass,
            mu=0.6, kc=0., dc=0.0, kb=0., db=0.0, robot.pinocchio_endeff_ids)

    robot.reset_state(q0, dq0)
    p.stepSimulation()

    for t in range(100):
        t = 0

        q, dq = robot.get_state_update_pinocchio()

        # Centroidal wrench from the plan.
        w_com = np.hstack([
            plan['centroidal_forces'][t],
            plan['centroidal_moments'][t]
        ])

        # Centroidal correction using basically PD controller.
        w_com += centr_controller.compute_com_wrench(t, q, dq,
            plan['com'][t], plan['vcom'][t],
            plan['q'][t][3:7], plan['base_ang_velocities'][t])

        # Compute the desired force at the endeffectors.
        F = centr_controller.compute_force_qp(t, q, dq, plan, w_com)

        # Compute torque using impedance controller.
        tau = solo_leg_ctrl.return_joint_torques(q, dq, kp, kd,
                                                plan['eff_positions'][t],
                                                plan['eff_velocities'][t], F)
        robot.send_joint_command(tau)

        p.stepSimulation()
        #time.sleep(0.0005)
