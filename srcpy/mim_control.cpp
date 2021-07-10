/**
 * @file
 * @license BSD 3-clause
 * @copyright Copyright (c) 2020, New York University and Max Planck
 * Gesellschaft
 *
 * @brief Python bindings for the mim_control package.
 */

#include <pybind11/pybind11.h>

namespace mim_control
{
void bind_impedance_controller(pybind11::module &module);
void bind_centroidal_pd_controller(pybind11::module &module);
void bind_centroidal_force_qp_controller(pybind11::module &module);
void bind_centroidal_impedance_controller(pybind11::module &module);

PYBIND11_MODULE(mim_control_cpp, m)
{
    m.doc() = R"pbdoc(
        mim_control python bindings
        ---------------------------------
        .. currentmodule:: mim_control
        .. autosummary::
           :toctree: _generate
           add
           subtract
    )pbdoc";

    bind_impedance_controller(m);
    bind_centroidal_pd_controller(m);
    bind_centroidal_force_qp_controller(m);
    bind_centroidal_impedance_controller(m);
}

}  // namespace mim_control
