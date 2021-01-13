/**
 * @file
 * @license BSD 3-clause
 * @copyright Copyright (c) 2020, New York University and Max Planck
 * Gesellschaft
 *
 * @brief Python bindings for the blmc_controllers package.
 */

#include <pybind11/pybind11.h>

namespace blmc_controllers{

void bind_impedance_controller(pybind11::module &module);
void bind_centroidal_pd_controller(pybind11::module &module);
void bind_centroidal_force_qp_controller(pybind11::module& module);

PYBIND11_MODULE(py_blmc_controllers, m) {
  m.doc() = R"pbdoc(
        blmc_controllers python bindings
        ---------------------------------
        .. currentmodule:: blmc_controllers
        .. autosummary::
           :toctree: _generate
           add
           subtract
    )pbdoc";

  bind_impedance_controller(m);
  bind_centroidal_pd_controller(m);
  bind_centroidal_force_qp_controller(m);
}

} // namespace blmc_controllers
