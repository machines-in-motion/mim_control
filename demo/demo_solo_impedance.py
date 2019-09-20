##################################################################################################################
## Demo file showing how to import and use the leg impedance control for solo 12 DOF
#################################################################################################################
## Author: Avadesh Meduri
## Date: 20/09/2019 
#################################################################################################################

import numpy as np

import time

import os
import rospkg
import pybullet as p
import pinocchio as se3

from robot_properties_solo.config import Solo12Config
from robot_properties_solo.quadruped12wrapper import Quadruped12Robot


# Create a robot instance. This initializes the simulator as well.
robot = Quadruped12Robot()
tau = np.zeros(12)

# Reset the robot to some initial state.
q0 = np.matrix(Solo12Config.initial_configuration).T
dq0 = np.matrix(Solo12Config.initial_velocity).T
robot.reset_state(q0, dq0)


# Run the simulator for 100 steps
for i in range(230):
    # TODO: Implement a controller here.
    robot.send_joint_command(tau)
    
    # Step the simulator.
    p.stepSimulation()
    # time.sleep(0.001) # You can sleep here if you want to slow down the replay

# Read the final state and forces after the stepping.
q, dq = robot.get_state()
active_eff, forces = robot.get_force()
print('q', q)
print('dq', dq)
print('active eff', active_eff)
print('forces', forces)