/**
 * @file
 * @license BSD 3-clause
 * @copyright Copyright (c) 2020, New York University and Max Planck
 * Gesellschaft
 *
 * @brief This is the implementation for impedance controller between any two
 * frames of the robot.
 *
 */

#pragma once

#include <sstream>

namespace mim_control
{
namespace dynamic_graph
{
std::string make_signal_string(const bool& is_input_signal,
                               const std::string& class_name,
                               const std::string& object_name,
                               const std::string& signal_type,
                               const std::string& signal_name);

// clang-format off
#define crop_underscore(var_name)                                              \
    std::string(#var_name).substr(0, std::string(#var_name).size() - 1)

#define define_input_signal(signal_var_name, signal_type)                      \
  signal_var_name(                                                             \
      NULL,                                                                    \
      make_signal_string(true, CLASS_NAME, name, signal_type,                  \
                         crop_underscore(signal_var_name)))

#define define_output_signal(                                                  \
    signal_var_name, signal_type, signal_dep, callback)                        \
  signal_var_name(                                                             \
      boost::bind(callback, this, _1, _2), signal_dep,                         \
      make_signal_string(false, CLASS_NAME, name, signal_type,                 \
                         crop_underscore(signal_var_name)))
// clang-format on

}  // namespace dynamic_graph
}  // namespace mim_control
