/**
 * @file
 * @license BSD 3-clause
 * @copyright Copyright (c) 2020, New York University and Max Planck
 * Gesellschaft
 *
 * @brief Dynamic graph wrapper around the CentroidalPDController class.
 *
 */

#pragma once

// clang-format off
#include "pinocchio/fwd.hpp"
// clang-format on
#include "dynamic-graph/all-signals.h"
#include "dynamic-graph/entity.h"
#include "mim_control/centroidal_pd_controller.hpp"
#include "mim_control/dynamic_graph/signal_utils.hpp"

namespace mim_control
{
namespace dynamic_graph
{
/**
 * @brief Entity around the CentroidalPDController class of this package.
 */
class CentroidalPDController : public dynamicgraph::Entity
{
    DYNAMIC_GRAPH_ENTITY_DECL();

public:
    /**
     * @brief Construct a new CentroidalPDController object using its Dynamic
     * Graph name.
     *
     * @param name
     */
    CentroidalPDController(const std::string& name);

    /**
     * @brief Initialize the internal data. None real-time safe method.
     *
     * @param pinocchio_model rigid body model of the robot
     * @param root_frame_name root frame name where the spring starts(Ex. Hip)
     * @param end_frame_name frame name where the spring ends(Ex. end effector)
     */
    void initialize(const double& mass, const dynamicgraph::Vector& inertia);

    /*
     * Input Signals.
     */

    /** @brief Proportional gain on the center of mass tracking error. */
    dynamicgraph::SignalPtr<dynamicgraph::Vector, int> kp_com_sin_;

    /** @brief Derivative gain on the center of mass tracking error. */
    dynamicgraph::SignalPtr<dynamicgraph::Vector, int> kd_com_sin_;

    /** @brief Proportional gain on the base orientation tracking error. */
    dynamicgraph::SignalPtr<dynamicgraph::Vector, int> kp_base_sin_;

    /** @brief Derivative gain on the base orientation tracking error. */
    dynamicgraph::SignalPtr<dynamicgraph::Vector, int> kd_base_sin_;

    /** @brief Current position of the center of mass. */
    dynamicgraph::SignalPtr<dynamicgraph::Vector, int> actual_com_position_sin_;

    /** @brief */
    dynamicgraph::SignalPtr<dynamicgraph::Vector, int>
        desired_com_position_sin_;

    /** @brief */
    dynamicgraph::SignalPtr<dynamicgraph::Vector, int> actual_com_velocity_sin_;

    /** @brief */
    dynamicgraph::SignalPtr<dynamicgraph::Vector, int>
        desired_com_velocity_sin_;

    /** @brief */
    dynamicgraph::SignalPtr<dynamicgraph::Vector, int>
        actual_base_orientation_sin_;

    /** @brief */
    dynamicgraph::SignalPtr<dynamicgraph::Vector, int>
        desired_base_orientation_sin_;

    /** @brief */
    dynamicgraph::SignalPtr<dynamicgraph::Vector, int>
        actual_base_angular_velocity_sin_;

    /** @brief */
    dynamicgraph::SignalPtr<dynamicgraph::Vector, int>
        desired_base_angular_velocity_sin_;

    /*
     * Output Signals.
     */

    /** @brief Output joint torques. */
    dynamicgraph::SignalTimeDependent<dynamicgraph::Vector, int> wrench_sout_;

protected:
    /**
     * @brief Callback function of the torque_sout_ signal.
     *
     * @param torque
     * @param time
     * @return dynamicgraph::Vector&
     */
    dynamicgraph::Vector& wrench_callback(dynamicgraph::Vector& signal_data,
                                          int time);

    /** @brief Actual controller class we wrap around. */
    mim_control::CentroidalPDController centroidal_pd_controller_;
};

}  // namespace dynamic_graph
}  // namespace mim_control
