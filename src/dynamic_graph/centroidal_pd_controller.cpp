/**
 * @file
 * @license BSD 3-clause
 * @copyright Copyright (c) 2020, New York University and Max Planck
 * Gesellschaft
 *
 * @brief Dynamic graph wrapper around the CentroidalPDController class.
 */

#include "mim_control/dynamic_graph/centroidal_pd_controller.hpp"

#include "dynamic-graph/all-commands.h"
#include "dynamic-graph/factory.h"

namespace mim_control
{
namespace dynamic_graph
{
using ::dynamicgraph::command::docCommandVoid2;
using ::dynamicgraph::command::makeCommandVoid2;

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(CentroidalPDController,
                                   "CentroidalPDController");

CentroidalPDController::CentroidalPDController(const std::string& name)
    :  // Inheritance.
      dynamicgraph::Entity(name),
      // Input signals.
      define_input_signal(kp_com_sin_, "Vector3d"),
      define_input_signal(kd_com_sin_, "Vector3d"),
      define_input_signal(kp_base_sin_, "Vector3d"),
      define_input_signal(kd_base_sin_, "Vector3d"),
      define_input_signal(actual_com_position_sin_, "Vector3d"),
      define_input_signal(desired_com_position_sin_, "Vector3d"),
      define_input_signal(actual_com_velocity_sin_, "Vector3d"),
      define_input_signal(desired_com_velocity_sin_, "Vector3d"),
      define_input_signal(actual_base_orientation_sin_, "Vector4d_quat"),
      define_input_signal(desired_base_orientation_sin_, "Vector4d_quat"),
      define_input_signal(actual_base_angular_velocity_sin_, "Vector3d"),
      define_input_signal(desired_base_angular_velocity_sin_, "Vector3d"),
      // Output signals.
      define_output_signal(
          wrench_sout_,
          "inner",
          kp_com_sin_ << kd_com_sin_ << kp_base_sin_ << kd_base_sin_
                      << actual_com_position_sin_ << desired_com_position_sin_
                      << actual_com_velocity_sin_ << desired_com_velocity_sin_
                      << actual_base_orientation_sin_
                      << desired_base_orientation_sin_
                      << actual_base_angular_velocity_sin_
                      << desired_base_angular_velocity_sin_,
          &CentroidalPDController::wrench_callback)
{
    signalRegistration(kp_com_sin_
                       << kd_com_sin_ << kp_base_sin_ << kd_base_sin_
                       << actual_com_position_sin_ << desired_com_position_sin_
                       << actual_com_velocity_sin_ << desired_com_velocity_sin_
                       << actual_base_orientation_sin_
                       << desired_base_orientation_sin_
                       << actual_base_angular_velocity_sin_
                       << desired_base_angular_velocity_sin_ << wrench_sout_);

    addCommand("initialize",
               makeCommandVoid2(
                   *this,
                   &CentroidalPDController::initialize,
                   docCommandVoid2("Initialize the CentroidalPDController.",
                                   "Robot total mass.",
                                   "Base inertia.")));
}

void CentroidalPDController::initialize(const double& mass,
                                        const dynamicgraph::Vector& inertia)
{
    centroidal_pd_controller_.initialize(mass, inertia);
}

dynamicgraph::Vector& CentroidalPDController::wrench_callback(
    dynamicgraph::Vector& signal_data, int time)
{
    const dynamicgraph::Vector& kp_com = kp_com_sin_.access(time);
    const dynamicgraph::Vector& kd_com = kd_com_sin_.access(time);
    const dynamicgraph::Vector& kp_base = kp_base_sin_.access(time);
    const dynamicgraph::Vector& kd_base = kd_base_sin_.access(time);
    const dynamicgraph::Vector& actual_com_position =
        actual_com_position_sin_.access(time);
    const dynamicgraph::Vector& desired_com_position =
        desired_com_position_sin_.access(time);
    const dynamicgraph::Vector& actual_com_velocity =
        actual_com_velocity_sin_.access(time);
    const dynamicgraph::Vector& desired_com_velocity =
        desired_com_velocity_sin_.access(time);
    const dynamicgraph::Vector& actual_base_orientation =
        actual_base_orientation_sin_.access(time);
    const dynamicgraph::Vector& desired_base_orientation =
        desired_base_orientation_sin_.access(time);
    const dynamicgraph::Vector& actual_base_angular_velocity =
        actual_base_angular_velocity_sin_.access(time);
    const dynamicgraph::Vector& desired_base_angular_velocity =
        desired_base_angular_velocity_sin_.access(time);

    centroidal_pd_controller_.run(kp_com,
                                  kd_com,
                                  kp_base,
                                  kd_base,
                                  actual_com_position,
                                  desired_com_position,
                                  actual_com_velocity,
                                  desired_com_velocity,
                                  actual_base_orientation,
                                  desired_base_orientation,
                                  actual_base_angular_velocity,
                                  desired_base_angular_velocity);

    signal_data = centroidal_pd_controller_.get_wrench();
    return signal_data;
}

}  // namespace dynamic_graph
}  // namespace mim_control
