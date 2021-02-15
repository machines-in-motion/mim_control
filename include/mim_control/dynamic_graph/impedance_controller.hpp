/**
 * @file
 * @license BSD 3-clause
 * @copyright Copyright (c) 2020, New York University and Max Planck
 * Gesellschaft
 *
 * @brief Dynamic graph wrapper around the ImpedanceController class.
 *
 */

#pragma once

// clang-format off
#include "pinocchio/fwd.hpp"
// clang-format on
#include "dynamic-graph/all-signals.h"
#include "dynamic-graph/entity.h"
#include "mim_control/impedance_controller.hpp"

namespace mim_control
{
namespace dynamic_graph
{
/**
 * @brief Entity around the ImpedanceController class of this package.
 */
class ImpedanceController : public dynamicgraph::Entity
{
    DYNAMIC_GRAPH_ENTITY_DECL();

public:
    /**
     * @brief Construct a new ImpedanceController object using its Dynamic Graph
     * name.
     *
     * @param name
     */
    ImpedanceController(const std::string& name);

    /**
     * @brief Initialize the internal data. None real-time safe method.
     *
     * @param pinocchio_model rigid body model of the robot
     * @param root_frame_name root frame name where the spring starts(Ex. Hip)
     * @param end_frame_name frame name where the spring ends(Ex. end effector)
     */
    void initialize(const pinocchio::Model& pinocchio_model,
                    const std::string& root_frame_name,
                    const std::string& end_frame_name);

    /*
     * Input Signals.
     */

    /** @brief Robot generalized coordinates (q). */
    dynamicgraph::SignalPtr<dynamicgraph::Vector, int> robot_configuration_sin_;

    /** @brief Robot generalized velocity (dq). */
    dynamicgraph::SignalPtr<dynamicgraph::Vector, int> robot_velocity_sin_;

    /** @brief Proportional gain from the cartesian PD. */
    dynamicgraph::SignalPtr<dynamicgraph::Vector, int> gain_proportional_sin_;

    /** @brief Proportional gain from the cartesian PD. */
    dynamicgraph::SignalPtr<dynamicgraph::Vector, int> gain_derivative_sin_;

    /** @brief Gain multiplying the feedforward force. */
    dynamicgraph::SignalPtr<double, int> gain_feed_forward_force_sin_;

    /** @brief */
    dynamicgraph::SignalPtr<dynamicgraph::Vector, int>
        desired_end_frame_placement_sin_;

    /** @brief */
    dynamicgraph::SignalPtr<dynamicgraph::Vector, int>
        desired_end_frame_velocity_sin_;

    /** @brief */
    dynamicgraph::SignalPtr<dynamicgraph::Vector, int> feed_forward_force_sin_;

    /*
     * Output Signals.
     */

    /** @brief Output joint torques. */
    dynamicgraph::SignalTimeDependent<dynamicgraph::Vector, int> torque_sout_;

    /** @brief Output generalized torques. */
    dynamicgraph::SignalTimeDependent<dynamicgraph::Vector, int>
        joint_torque_sout_;

    /** @brief Impedance forces computed. */
    dynamicgraph::SignalTimeDependent<dynamicgraph::Vector, int>
        impedance_force_;

    /** @brief Internal transition signal. */
    dynamicgraph::SignalTimeDependent<bool, int> one_iteration_sout_;

protected:
    /**
     * @brief Callback function of the torque_sout_ signal.
     *
     * @param torque
     * @param time
     * @return dynamicgraph::Vector&
     */
    dynamicgraph::Vector& torque_callback(dynamicgraph::Vector& torque,
                                          int time);

    /**
     * @brief Callback function of the joint_torque_sout_ signal.
     *
     * @param signal_data
     * @param time
     * @return dynamicgraph::Vector&
     */
    dynamicgraph::Vector& joint_torque_callback(
        dynamicgraph::Vector& signal_data, int time);

    /**
     * @brief Callback function of the impedance_force_sout_ signal.
     *
     * @param signal_data
     * @param time
     * @return dynamicgraph::Vector&
     */
    dynamicgraph::Vector& impedance_force_callback(
        dynamicgraph::Vector& signal_data, int time);

    /**
     * @brief Internally calls the ImpedanceController class.
     *
     * @param signal_data
     * @param time
     * @return true
     * @return false
     */
    bool& one_iteration_callback(bool& signal_data, int time);

    /** @brief Actual controller class we wrap around. */
    mim_control::ImpedanceController impedance_controller_;

    /** @brief Used for type conversion. */
    pinocchio::SE3 desired_end_frame_placement_;

    /** @brief Used for type conversion. */
    pinocchio::Motion desired_end_frame_velocity_;

    /** @brief Used for type conversion. */
    pinocchio::Force feed_forward_force_;
};

}  // namespace dynamic_graph
}  // namespace mim_control
