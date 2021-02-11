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

#include "mim_control/dynamic_graph/signal_utils.hpp"

namespace mim_control
{
namespace dynamic_graph
{
std::string make_signal_string(const bool& is_input_signal,
                               const std::string& class_name,
                               const std::string& object_name,
                               const std::string& signal_type,
                               const std::string& signal_name)
{
    std::ostringstream oss;
    oss << class_name << "(" << object_name
        << ")::" << (is_input_signal ? "input" : "output") << "(" << signal_type
        << ")::" << signal_name;
    return oss.str();
}

}  // namespace dynamic_graph
}  // namespace mim_control
