#!/usr/bin/env python

""" Demos of the C++ Impedance controller.

License BSD 3-clause
Copyright (c) 2020, New York University and Max Planck Gesellschaft
"""

import time
import numpy as np
import pybullet
import pinocchio
# the robot model
from robot_properties_solo.config import Solo12Config
# the simulator
from robot_properties_solo.quadruped12wrapper import Solo12Robot
# the controller
from blmc_controllers import ImpedanceController

if __name__ == "__main__":
    # simulator
    simulator = Solo12Robot()

    # 1 controller per leg
    robot_config = Solo12Config()
    model = robot_config.robot_model
    world_frame_name = "universe"
    leg_names = ['FL', 'FR', 'HL', 'HR']
    hip_name = 'HFE'
    foot_name = 'FOOT'
    root_frame_names = [leg_name + '_' + hip_name for leg_name in leg_names]
    end_frame_names = [leg_name + '_' + foot_name for leg_name in leg_names]
    impedance_controllers = [ImpedanceController() for _ in leg_names]
    for imp_ctrl, root_frame_name, end_frame_name in zip(
            impedance_controllers, root_frame_names, end_frame_names):
        imp_ctrl.initialize(model, root_frame_name, end_frame_name)

    # Time variying parameters
    desired_end_frame_placement = pinocchio.SE3.Identity()
    desired_end_frame_velocity = pinocchio.Motion.Zero()
    feed_forward_force = pinocchio.Force.Zero()
    robot_configuration = np.zeros(model.nq)
    robot_velocity = np.zeros(model.nv)
    gain_proportional = np.zeros(6)
    gain_derivative = np.zeros(6)
    tau = np.zeros(model.nv)
    gain_feed_forward_force = 0.0

    for i in range(1000000):
        # Step the simulator.
        pybullet.stepSimulation()
        time.sleep(0.001)

        # Get the current state.
        robot_configuration, robot_velocity = simulator.get_state()

        # Update the desired trajectory and compute the control.
        for imp_ctrl in impedance_controllers:
            desired_end_frame_placement.translation[:] = [0.0, 0.0, -0.25]
            desired_end_frame_velocity.setZero()
            feed_forward_force.setZero()
            feed_forward_force.linear[:] = [
                0.0, 0.0, (robot_config.mass * 9.8) / 4]
            gain_proportional[:] = [50, 50, 50, 0, 0, 0]
            gain_derivative[:] = [10.0, 10.0, 10.0, 0, 0, 0]

            imp_ctrl.run(robot_configuration,
                         robot_velocity,
                         gain_proportional,
                         gain_derivative,
                         gain_feed_forward_force,
                         desired_end_frame_placement,
                         desired_end_frame_velocity,
                         feed_forward_force)
        tau.fill(0)
        for imp_ctrl in [impedance_controllers[0]]: #impedance_controllers:
            tau[:] += imp_ctrl.get_torques()
        
        # Send the control.
        simulator.send_joint_command(tau[6:])
