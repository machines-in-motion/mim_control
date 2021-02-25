/**
 * @file
 * @license BSD 3-clause
 * @copyright Copyright (c) 2020, New York University and Max Planck
 * Gesellschaft
 *
 * @brief Dynamic graph wrapper around the CentroidalForceQPController class.
 */

#include "mim_control/dynamic_graph/centroidal_force_qp_controller.hpp"
#include "dynamic-graph/all-commands.h"
#include "dynamic-graph/factory.h"
#include "mim_control/dynamic_graph/signal_utils.hpp"

namespace mim_control
{
namespace dynamic_graph
{
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(CentroidalForceQPController,
                                   "CentroidalForceQPController");

CentroidalForceQPController::CentroidalForceQPController(
    const std::string& name)
    :  // Inheritance.
      dynamicgraph::Entity(name),
      // Input signals.
      define_input_signal(w_com_sin_, "Vector3d"),
      define_input_signal(relative_position_endeff_sin_, "Vector(nb_ee_x_3)d"),
      define_input_signal(cnt_array_sin_, "Vector(nb_ee)d"),
      // Output signals.
      define_output_signal(
          forces_sout_,
          "Vector(nb_ee_x_3)d",
          w_com_sin_ << relative_position_endeff_sin_ << cnt_array_sin_,
          &CentroidalForceQPController::forces_callback)
{
    signalRegistration(w_com_sin_ << relative_position_endeff_sin_
                                  << cnt_array_sin_ << forces_sout_);
}

void CentroidalForceQPController::initialize(
    int number_endeffectors,
    double friction_coeff,
    Eigen::Ref<const Vector6d> qp_penalty_weights)
{
    assert(qp_penalty_weights.size() == 6 &&
           "Wrong size for qp_penalty_weights, expected dimension 6.");

    force_ctrl_.initialize(
        number_endeffectors, friction_coeff, qp_penalty_weights);
}

dynamicgraph::Vector& CentroidalForceQPController::forces_callback(
    dynamicgraph::Vector& signal_data, int time)
{
    const dynamicgraph::Vector& w_com = w_com_sin_.access(time);
    const dynamicgraph::Vector& relative_position_endeff =
        relative_position_endeff_sin_.access(time);
    const dynamicgraph::Vector& cnt_array = cnt_array_sin_.access(time);

    assert(w_com.size() == 6 && "Wrong size of CoM wrench, expected 6.");
    assert(relative_position_endeff.size() == force_ctrl_.get_nb_eff() * 3 &&
           "Wrong size of EE position, expected nb_eff * 3.");
    assert(cnt_array.size() == force_ctrl_.get_nb_eff() &&
           "Wrong size of contact array.");

    force_ctrl_.run(w_com, relative_position_endeff, cnt_array);

    signal_data = force_ctrl_.get_forces();
    return signal_data;
}

}  // namespace dynamic_graph
}  // namespace mim_control
