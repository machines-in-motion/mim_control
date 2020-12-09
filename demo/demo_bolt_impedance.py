##################################################################################################################
## Demo file showing how to import and use the leg impedance control for bolt
#################################################################################################################
## Author: Avadesh Meduri
## Date: 9/4/2020
#################################################################################################################

import numpy as np
import time

import pybullet as p
import pinocchio as se3

from robot_properties_bolt.config import BoltConfig
from robot_properties_bolt.bolt_wrapper import BoltRobot

from blmc_controllers.bolt_impedance_controller import BoltImpedanceController


robot = BoltRobot()
tau = np.zeros(6)

# Reset the robot to some initial state.
q0 = np.matrix(BoltConfig.initial_configuration).T
dq0 = np.matrix(BoltConfig.initial_velocity).T
robot.reset_state(q0, dq0)

###########################################################################

x_des = 2*[0.0, 0.0, -0.25]
xd_des = 2*[0,0,0]
kp = 2 * [200,200,200]
kd = 2 * [10.0,10.0,10.0]
f = np.zeros(6)

###########################################################################

bolt_leg_ctrl = BoltImpedanceController(robot)

for i in range(4000):
    p.stepSimulation()
    time.sleep(0.001)
    q, dq = robot.get_state()
    tau = bolt_leg_ctrl.return_joint_torques(q,dq,kp,kd,x_des,xd_des,f)
    robot.send_joint_command(tau)

