/**
 * @file
 * @license BSD 3-clause
 * @copyright Copyright (c) 2020, New York University and Max Planck
 * Gesellschaft
 *
 * @brief Python bindings for the StepperHead class
 */

#include "mim_control/impedance_controller3d.hpp"
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <boost/python.hpp>

namespace py = pybind11;

namespace mim_control
{
void bind_impedance_controller3d(py::module& module)
{
    py::class_<ImpedanceController3D>(module, "ImpedanceController3D")
        .def(py::init<>())

        // Public methods.
        .def("initialize",
             [](ImpedanceController3D& obj,
                py::object model,
                const std::string& root_frame_name,
                const std::string& end_frame_name) {
                 const pinocchio::Model& pinocchio_model =
                     boost::python::extract<const pinocchio::Model&>(
                         model.ptr());
                 obj.initialize(
                     pinocchio_model, root_frame_name, end_frame_name);
                 return;
             })
        .def("run",
             [](ImpedanceController3D& obj,
                const Eigen::VectorXd& robot_configuration,
                const Eigen::VectorXd& robot_velocity,
                const Eigen::Array3d& gain_proportional,
                const Eigen::Array3d& gain_derivative,
                const double& gain_feed_forward_force,
                py::object py_desired_end_frame_placement,
                py::object py_desired_end_frame_velocity,
                py::object py_feed_forward_force) {
                 const pinocchio::SE3& desired_end_frame_placement =
                     boost::python::extract<const pinocchio::SE3&>(
                         py_desired_end_frame_placement.ptr());
                 const pinocchio::Motion& desired_end_frame_velocity =
                     boost::python::extract<const pinocchio::Motion&>(
                         py_desired_end_frame_velocity.ptr());
                 const pinocchio::Force& feed_forward_force =
                     boost::python::extract<const pinocchio::Force&>(
                         py_feed_forward_force.ptr());
                 obj.run(robot_configuration,
                         robot_velocity,
                         gain_proportional,
                         gain_derivative,
                         gain_feed_forward_force,
                         desired_end_frame_placement,
                         desired_end_frame_velocity,
                         feed_forward_force);
                 return;
             })
        .def("get_torques",
             &ImpedanceController3D::get_torques,
             py::return_value_policy::reference)
        .def("get_joint_torques",
             &ImpedanceController3D::get_joint_torques,
             py::return_value_policy::reference)
        .def("get_impedance_force",
             &ImpedanceController3D::get_impedance_force,
             py::return_value_policy::reference);
}

}  // namespace mim_control