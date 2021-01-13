/**
 * @file
 * @license BSD 3-clause
 * @copyright Copyright (c) 2020, New York University and Max Planck
 * Gesellschaft
 *
 * @brief Python bindings for the CentroidalForceQPController class.
 */

#include "blmc_controllers/centroidal_force_qp_controller.hpp"
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>

namespace py = pybind11;

namespace blmc_controllers{

void bind_centroidal_force_qp_controller(py::module& module)
{
    py::class_<CentroidalForceQPController>(module, "CentroidalForceQPController")
        .def(py::init<>())

        // Public methods.
        .def("initialize",  &CentroidalForceQPController::initialize)
        .def("run",  &CentroidalForceQPController::run)
        .def("get_forces",
             &CentroidalForceQPController::get_forces,
             py::return_value_policy::reference)
        ;
}

} // blmc_controllers
