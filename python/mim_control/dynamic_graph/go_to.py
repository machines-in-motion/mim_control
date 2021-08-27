"""go_to

Dynamic graph sub-graph implementing a "go to" controller at the joint level.

License BSD-3-Clause
Copyright (c) 2021, New York University and Max Planck Gesellschaft.
"""

import numpy as np
import dynamic_graph as dg
from dynamic_graph.sot.core.smooth_reach import SmoothReach
from dynamic_graph.sot.core.control_pd import ControlPD


class GoTo(object):
    """
    This class allow you to specify a configuration that the robot will reach
    in a certain amount of iteration (time = iteration * sampling_period).
    """

    def __init__(self, nb_dof, prefix=""):
        self._nb_dof = nb_dof
        self._prefix = prefix

        # Euclidean interpolation from 2 vectors.
        self._smooth_reach = SmoothReach(self._prefix + "go_to_smooth_reach")

        # PD controller.
        self._pd_ctrl = ControlPD(self._prefix + "go_to_pd_controller")
        dg.plug(self._smooth_reach.goal, self._pd_ctrl.desired_position)
        self._pd_ctrl.desired_velocity.value = np.array(nb_dof * [0.0])

    def set_pd_gains(self, Kp, Kd):
        """
        Set the PD controller gains
        """
        self._pd_ctrl.Kp.value = np.array(self._nb_dof * [Kp])
        self._pd_ctrl.Kd.value = np.array(self._nb_dof * [Kd])

    def go_to(self, desired_joint_position_rad, nb_iteration):
        """ Set the curve goal. set(vector (goal), int (duration)) """
        if np.array(desired_joint_position_rad).size != self._nb_dof:
            print(
                "Warning: Wrong number of input desired joint positions, ",
                "nothing to be done",
            )
            return
        else:
            return self._smooth_reach.set(
                np.array(desired_joint_position_rad), nb_iteration
            )

    def freeze(self):
        self.go_to(self._pd_ctrl.position.value, 1)

    def record_data(self, robot):
        # Adding logging traces.
        robot.add_trace(self._pd_ctrl.name, "desired_position")
        robot.add_trace(self._pd_ctrl.name, "desired_velocity")
        robot.add_trace(self._pd_ctrl.name, "position")
        robot.add_trace(self._pd_ctrl.name, "velocity")
        robot.add_trace(self._pd_ctrl.name, "control")
        robot.add_trace(self._smooth_reach.name, "start")
        robot.add_trace(self._smooth_reach.name, "goal")

    def plug(self, joint_positions_sout, joint_velocities_sout, torques_sin):
        # plug the inputs.
        dg.plug(joint_positions_sout, self._pd_ctrl.position)
        dg.plug(joint_velocities_sout, self._pd_ctrl.velocity)
        dg.plug(joint_positions_sout, self._smooth_reach.start)

        # plug the outins
        dg.plug(self._pd_ctrl.control, torques_sin)
