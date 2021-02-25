/**
 * @file
 * @license BSD 3-clause
 * @copyright Copyright (c) 2020, New York University and Max Planck
 * Gesellschaft
 *
 * @brief Expose the Device and the periodic call to python.
 */

#include "dynamic-graph/python/module.hh"
#include "dynamic-graph/python/signal.hh"
#include "mim_control/dynamic_graph/centroidal_force_qp_controller.hpp"
#include "mim_control/dynamic_graph/centroidal_pd_controller.hpp"
#include "mim_control/dynamic_graph/impedance_controller.hpp"

namespace dg = dynamicgraph;

typedef bp::return_value_policy<bp::reference_existing_object>
    reference_existing_object;

BOOST_PYTHON_MODULE(wbc)
{
    bp::import("dynamic_graph");

    using mim_control::dynamic_graph::ImpedanceController;
    dynamicgraph::python::exposeEntity<ImpedanceController>().def(
        "initialize",
        +[](ImpedanceController& ImpedanceController,
            const boost::python::object& pinocchio_model,
            const std::string& root_frame_name,
            const std::string& end_frame_name) {
            const pinocchio::Model& pinocchio_model_ref =
                boost::python::extract<const pinocchio::Model&>(
                    pinocchio_model);
            ImpedanceController.initialize(
                pinocchio_model_ref, root_frame_name, end_frame_name);
            return;
        },
        "Initialize the ImpedanceController.");

    using mim_control::dynamic_graph::CentroidalPDController;
    dynamicgraph::python::exposeEntity<CentroidalPDController>();

    using mim_control::dynamic_graph::CentroidalForceQPController;
    dynamicgraph::python::exposeEntity<CentroidalForceQPController>().def(
        "initialize",
        &CentroidalForceQPController::initialize,
        "Initialize the CentroidalForceQPController.");
}
