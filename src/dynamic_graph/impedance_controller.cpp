/**
 * @file
 * @license BSD 3-clause
 * @copyright Copyright (c) 2020, New York University and Max Planck
 * Gesellschaft
 *
 * @brief Dynamic graph wrapper around the ImpedanceController class.
 */

#include "mim_control/dynamic_graph/impedance_controller.hpp"

#include "Eigen/Eigen"
#include "dynamic-graph/factory.h"
#include "mim_control/dynamic_graph/signal_utils.hpp"

namespace mim_control
{
namespace dynamic_graph
{
typedef Eigen::Map<const pinocchio::SE3::Quaternion> QuatConstMap;

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(ImpedanceController, "ImpedanceController");

ImpedanceController::ImpedanceController(const std::string& name)
    :  // Inheritance.
      dynamicgraph::Entity(name),
      // Input signals.
      define_input_signal(robot_configuration_sin_, "VectorXd"),
      define_input_signal(robot_velocity_sin_, "VectorXd"),
      define_input_signal(gain_proportional_sin_, "Vector6d"),
      define_input_signal(gain_derivative_sin_, "Vector6d"),
      define_input_signal(gain_feed_forward_force_sin_, "double"),
      define_input_signal(desired_end_frame_placement_sin_, "Vector7d_XYZQuat"),
      define_input_signal(desired_end_frame_velocity_sin_, "Vector6d_Motion"),
      define_input_signal(feed_forward_force_sin_, "Vector6d_Force"),
      // Output signals.
      define_output_signal(torque_sout_,
                           "inner",
                           one_iteration_sout_,
                           &ImpedanceController::torque_callback),
      define_output_signal(joint_torque_sout_,
                           "inner",
                           one_iteration_sout_,
                           &ImpedanceController::joint_torque_callback),
      define_output_signal(impedance_force_,
                           "inner",
                           one_iteration_sout_,
                           &ImpedanceController::impedance_force_callback),
      // Inner signal.
      one_iteration_sout_(
          boost::bind(
              &ImpedanceController::one_iteration_callback, this, _1, _2),
          robot_configuration_sin_
              << robot_velocity_sin_ << gain_proportional_sin_
              << gain_derivative_sin_ << gain_feed_forward_force_sin_
              << desired_end_frame_placement_sin_
              << desired_end_frame_velocity_sin_ << feed_forward_force_sin_,
          make_signal_string(
              false, CLASS_NAME, name, "bool", "one_iteration_sout"))
{
    signalRegistration(
        robot_configuration_sin_
        << robot_velocity_sin_ << gain_proportional_sin_ << gain_derivative_sin_
        << gain_feed_forward_force_sin_ << desired_end_frame_placement_sin_
        << desired_end_frame_velocity_sin_ << feed_forward_force_sin_
        << torque_sout_ << joint_torque_sout_ << impedance_force_);
}

void ImpedanceController::initialize(const pinocchio::Model& pinocchio_model,
                                     const std::string& root_frame_name,
                                     const std::string& end_frame_name)
{
    impedance_controller_.initialize(
        pinocchio_model, root_frame_name, end_frame_name);
}

dynamicgraph::Vector& ImpedanceController::torque_callback(
    dynamicgraph::Vector& signal_data, int time)
{
    one_iteration_sout_.access(time);
    signal_data = impedance_controller_.get_torques();
    return signal_data;
}

dynamicgraph::Vector& ImpedanceController::joint_torque_callback(
    dynamicgraph::Vector& signal_data, int time)
{
    one_iteration_sout_.access(time);
    signal_data = impedance_controller_.get_joint_torques();
    return signal_data;
}

dynamicgraph::Vector& ImpedanceController::impedance_force_callback(
    dynamicgraph::Vector& signal_data, int time)
{
    one_iteration_sout_.access(time);
    signal_data = impedance_controller_.get_impedance_force();
    return signal_data;
}

bool& ImpedanceController::one_iteration_callback(bool& signal_data, int time)
{
    const dynamicgraph::Vector& robot_configuration =
        robot_configuration_sin_.access(time);
    const dynamicgraph::Vector& robot_velocity =
        robot_velocity_sin_.access(time);
    const dynamicgraph::Vector& gain_proportional =
        gain_proportional_sin_.access(time);
    const dynamicgraph::Vector& gain_derivative =
        gain_derivative_sin_.access(time);
    const double& gain_feed_forward_force =
        gain_feed_forward_force_sin_.access(time);
    const dynamicgraph::Vector& desired_end_frame_placement =
        desired_end_frame_placement_sin_.access(time);
    const dynamicgraph::Vector& desired_end_frame_velocity =
        desired_end_frame_velocity_sin_.access(time);
    const dynamicgraph::Vector& feed_forward_force =
        feed_forward_force_sin_.access(time);

    QuatConstMap quat(desired_end_frame_placement.tail<4>().data());
    desired_end_frame_placement_.rotation() = quat.matrix();
    desired_end_frame_placement_.translation() =
        desired_end_frame_placement.template head<3>();

    desired_end_frame_velocity_.toVector() = desired_end_frame_velocity;
    feed_forward_force_.toVector() = feed_forward_force;

    impedance_controller_.run(robot_configuration,
                              robot_velocity,
                              gain_proportional.array(),
                              gain_derivative.array(),
                              gain_feed_forward_force,
                              desired_end_frame_placement_,
                              desired_end_frame_velocity_,
                              feed_forward_force_);
    signal_data = true;
    return signal_data;
}

}  // namespace dynamic_graph
}  // namespace mim_control
