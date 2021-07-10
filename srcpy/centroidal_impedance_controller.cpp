/**
 * @file
 * @license BSD 3-clause
 * @copyright Copyright (c) 2020, New York University and Max Planck
 * Gesellschaft
 *
 * @brief Python bindings for the CentroidalPDController class.
 */

#include "mim_control/centroidal_impedance_controller.hpp"
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pybind11/pybind11.h>
#include <boost/python.hpp>

namespace py = pybind11;

namespace mim_control
{
void bind_centroidal_impedance_controller(py::module& module)
{
    py::class_<CentroidalImpedanceController>(module, "CentroidalImpedanceController")
        .def(py::init<>())

        // Public methods.
        // .def("initialize", &CentroidalImpedanceController::initialize)
        .def("initialize",
             [](CentroidalImpedanceController& obj,
                const double& mass,
                Eigen::Ref<const Eigen::Vector3d> inertia,
                py::object model,
                const std::string& root_frame_name,
                const std::vector<std::string>& end_frame_names,
                double friction_coeff,
                Eigen::Ref<const Vector6d> qp_penalty_weights,
                Eigen::Ref<const Eigen::Vector3d> kc,
                Eigen::Ref<const Eigen::Vector3d> dc,
                Eigen::Ref<const Eigen::Vector3d> kb,
                Eigen::Ref<const Eigen::Vector3d> db,
                Eigen::Ref<const Array6d> frame_placement_error_gain,
                Eigen::Ref<const Array6d> frame_velocity_error_gain) {
                 const pinocchio::Model& pinocchio_model =
                     boost::python::extract<const pinocchio::Model&>(
                         model.ptr());
                 obj.initialize(
                     mass, inertia, pinocchio_model, root_frame_name,
                     end_frame_names, friction_coeff, qp_penalty_weights,
                     kc, dc, kb, db,
                     frame_placement_error_gain, frame_velocity_error_gain);
                 return;
             })

        .def("update_centroidal_gains", &CentroidalImpedanceController::update_centroidal_gains)
        .def("update_endeff_gains", &CentroidalImpedanceController::update_centroidal_gains)
        .def("run", &CentroidalImpedanceController::run)
        .def("get_joint_torques",
             &CentroidalImpedanceController::get_joint_torques,
             py::return_value_policy::reference);
}

}  // namespace mim_control
