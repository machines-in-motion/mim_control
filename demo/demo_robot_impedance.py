##################################################################################################################
## Demo file creates impedance controllers between different frames  
## based on the input yaml file
#################################################################################################################
## Author: Avadesh Meduri & Paarth Shah
## Date: 9/12/2020
#################################################################################################################

import numpy as np
import time
import os

from blmc_controllers.robot_impedance_controller import RobotImpedanceController
from bullet_utils.env import BulletEnvWithGround
from robot_properties_solo.solo12wrapper import Solo12Robot, Solo12Config
from robot_properties_bolt.bolt_wrapper import BoltRobot, BoltConfig

# Create a Pybullet simulation environment
env = BulletEnvWithGround()
solo = False
bolt = True
# Create a robot instance. This initializes the simulator as well.
if solo:
    robot = env.add_robot(Solo12Robot)
    RobotConfig = Solo12Config
elif bolt:
    robot = env.add_robot(BoltRobot)
    RobotConfig = BoltConfig


tau = np.zeros(robot.nb_dof)

# # Reset the robot to some initial state.
q0 = np.matrix(RobotConfig.initial_configuration).T
dq0 = np.matrix(RobotConfig.initial_velocity).T

q0 = np.matrix(RobotConfig.initial_configuration).T
dq0 = np.matrix(RobotConfig.initial_velocity).T

robot.reset_state(q0, dq0)

# ###################### impedance controller demo #################################

x_des = robot.nb_ee*[0.0, 0.0, -0.25]
xd_des = robot.nb_ee*[0,0,0]
kp = robot.nb_ee * [200,200,200]
kd = robot.nb_ee * [10.0,10.0,10.0]
f = np.zeros(robot.nb_ee*3)
f = robot.nb_ee*[0.0, 0.0, (2.2*9.8)/4]
# ##################################################################################

# config_file = "./solo_impedance.yaml"
config_file = RobotConfig.paths["imp_ctrl_yaml"]
robot_leg_ctrl = RobotImpedanceController(robot, config_file)

# # Run the simulator for 100 steps
for i in range(4000):
    # TODO: Implement a controller here.
    # Step the simulator.
    env.step(sleep=True) # You can sleep here if you want to slow down the replay
    # Read the final state and forces after the stepping.
    q, dq = robot.get_state()
    tau = robot_leg_ctrl.return_joint_torques(q,dq,kp,kd,x_des,xd_des,f)
    robot.send_joint_command(tau)