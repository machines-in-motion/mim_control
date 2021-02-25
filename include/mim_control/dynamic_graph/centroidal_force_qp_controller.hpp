/**
 * @file
 * @license BSD 3-clause
 * @copyright Copyright (c) 2020, New York University and Max Planck
 * Gesellschaft
 *
 * @brief Dynamic graph wrapper around the CentroidalForceQPController class.
 *
 */

#pragma once

// clang-format off
#include "pinocchio/fwd.hpp"
// clang-format on
#include "dynamic-graph/all-signals.h"
#include "dynamic-graph/entity.h"
#include "mim_control/centroidal_force_qp_controller.hpp"

namespace mim_control
{
namespace dynamic_graph
{
/**
 * @brief Entity around the CentroidalForceQPController class of this package.
 */
class CentroidalForceQPController : public dynamicgraph::Entity
{
    DYNAMIC_GRAPH_ENTITY_DECL();

public:
    /**
     * @brief Construct a new CentroidalForceQPController object using its
     * Dynamic Graph name.
     *
     * @param name
     */
    CentroidalForceQPController(const std::string& name);

    /**
     * @brief Initialize the internal data. None real-time safe method.
     *
     * @param number_endeffectors Maximum number of endeffectors in the problem.
     * @param friction_coeff Floor friction coefficient to use.
     * @param qp_penalty_weights The penalty weight for the linear and angular
     * wcom violation.
     */
    void initialize(int number_endeffectors,
                    double friction_coeff,
                    Eigen::Ref<const Vector6d> qp_penalty_weights);

    /*
     * Input Signals.
     */

    /** @brief Robot generalized coordinates (q). */
    dynamicgraph::SignalPtr<dynamicgraph::Vector, int> w_com_sin_;

    /** @brief Robot generalized velocity (dq). */
    dynamicgraph::SignalPtr<dynamicgraph::Vector, int>
        relative_position_endeff_sin_;

    /** @brief Proportional gain from the cartesian PD. */
    dynamicgraph::SignalPtr<dynamicgraph::Vector, int> cnt_array_sin_;

    /*
     * Output Signals.
     */

    /** @brief Output joint torques. */
    dynamicgraph::SignalTimeDependent<dynamicgraph::Vector, int> forces_sout_;

protected:
    /**
     * @brief Callback function of the forces_sout_ signal.
     *
     * @param torque
     * @param time
     * @return dynamicgraph::Vector&
     */
    dynamicgraph::Vector& forces_callback(dynamicgraph::Vector& torque,
                                          int time);

    /** @brief Actual controller class we wrap around. */
    mim_control::CentroidalForceQPController force_ctrl_;
};

}  // namespace dynamic_graph
}  // namespace mim_control
