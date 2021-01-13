/**
 * @file
 * @license BSD 3-clause
 * @copyright Copyright (c) 2020, New York University and Max Planck
 * Gesellschaft
 *
 * @brief Python bindings for the CentroidalPDController class.
 */

#include "blmc_controllers/centroidal_pd_controller.hpp"
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>

namespace py = pybind11;

namespace blmc_controllers{

void bind_centroidal_pd_controller(py::module& module)
{
    py::class_<CentroidalPDController>(module, "CentroidalPDController")
        .def(py::init<>())

        // Public methods.
        .def("initialize",  &CentroidalPDController::initialize)
        .def("run",  &CentroidalPDController::run)
        .def("get_wrench",
             &CentroidalPDController::get_wrench,
             py::return_value_policy::reference)
        ;
}

} // blmc_controllers
