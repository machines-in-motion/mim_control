##################################################################################################################
## This file is the centroidal controller demo for a given robot (solo in this case)
#################################################################################################################
## Author: Avadesh Meduri
## Date: 9/12/2020
#################################################################################################################

import numpy as np
import time


from blmc_controllers.robot_impedance_controller import RobotImpedanceController
from blmc_controllers.robot_centroidal_controller import RobotCentroidalController
from bullet_utils.env import BulletEnvWithGround
from robot_properties_solo.solo12wrapper import Solo12Robot, Solo12Config


# Create a Pybullet simulation environment
env = BulletEnvWithGround()

# Create a robot instance. This initializes the simulator as well.
robot = env.add_robot(Solo12Robot)
tau = np.zeros(robot.nb_dof)

# Move the default position closer to the ground.
initial_configuration = [0., 0., 0.21, 0., 0., 0., 1.] + 4 * [0., 0.9, -1.8]
Solo12Config.initial_configuration = initial_configuration

# # Reset the robot to some initial state.
q0 = np.matrix(Solo12Config.initial_configuration).T
dq0 = np.matrix(Solo12Config.initial_velocity).T
robot.reset_state(q0, dq0)

x_com = [0.0, 0.0, 0.18]
xd_com = [0.0, 0.0, 0.0]

x_ori = [0., 0., 0., 1.]
x_angvel = [0., 0., 0.]
cnt_array = [1, 1, 1, 1]

# Impedance controller gains
kp = 4 * [0., 0., 0.] # Disable for now
kd = 4 * [0., 0., 0.]
x_des = 4 * [0., 0., -0.25] # Desired leg length
xd_des = 4 * [0. ,0., 0.]


## initialising controllers ############################

config_file = "./solo_impedance.yaml"
solo_cent_ctrl = RobotCentroidalController(robot,
        mu=0.6, kc=[200,200,200], dc=[5,5,5], kb=[200,200,200], db=[1.,1.,1.])
solo_leg_ctrl = RobotImpedanceController(robot, config_file)

# # Run the simulator for 100 steps
for i in range(4000):
    # Step the simulator.
    env.step(sleep=True) # You can sleep here if you want to slow down the replay
    # Read the final state and forces after the stepping.
    q, dq = robot.get_state()
    # computing forces to be applied in the centroidal space
    w_com = solo_cent_ctrl.compute_com_wrench(q, dq, x_com, xd_com, x_ori, x_angvel)
    # distrubuting forces to the active end effectors
    F = solo_cent_ctrl.compute_force_qp(q, dq, cnt_array, w_com)
    # passing forces to the impedance controller
    tau = solo_leg_ctrl.return_joint_torques(q,dq,kp,kd,x_des,xd_des,F)
    # passing torques to the robot
    robot.send_joint_command(tau)