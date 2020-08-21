#!/usr/bin/env python

""" Demos of the C++ Impedance controller.

License BSD 3-clause
Copyright (c) 2020, New York University and Max Planck Gesellschaft
"""

import numpy as np
import pinocchio
from blmc_controllers import ImpedanceController


if __name__ == "__main__":
    # initial paramters
    model = pinocchio.buildSampleModelHumanoid()
    root_frame_name = "lleg_shoulder1_joint"
    # If you to express every quantity in the world frame.
    # root_frame_name = "universe"
    end_frame_name = "lleg_wrist2_joint"

    # Time variying parameters
    desired_end_frame_placement = pinocchio.SE3.Random()
    desired_end_frame_velocity = pinocchio.Motion.Random()
    feed_forward_force = pinocchio.Force.Random()
    robot_configuration = np.zeros((model.nq, 1))
    robot_velocity = np.zeros((model.nv, 1))
    gain_proportional = np.zeros((6, 1))
    gain_derivative = np.zeros((6, 1))
    gain_feed_forward_force = 1.0

    imp = ImpedanceController()
    imp.initialize(model, root_frame_name, end_frame_name)
    imp.run(robot_configuration,
            robot_velocity,
            gain_proportional,
            gain_derivative,
            gain_feed_forward_force,
            desired_end_frame_placement,
            desired_end_frame_velocity,
            feed_forward_force)

    print(imp.get_torques())
    print(imp.get_impedance_force())