/**
 * @file
 * @license BSD 3-clause
 * @copyright Copyright (c) 2020, New York University and Max Planck
 * Gesellschaft
 *
 * @brief Implementation of the ImpedanceController class.
 */

#include "blmc_controllers/impedance_controller.hpp"
#include "pinocchio/algorithm/frames.hpp"

namespace blmc_controllers
{
ImpedanceController::ImpedanceController()
{
}

void ImpedanceController::initialize(const pinocchio::Model& pinocchio_model,
                                     const std::string& root_frame_name,
                                     const std::string& end_frame_name)
{
    // Copy the arguments internally.
    pinocchio_model_ = pinocchio_model;
    root_frame_name_ = root_frame_name;
    end_frame_name_ = end_frame_name;

    // Create the cache of the rigid body dynamics algorithms
    pinocchio_data_ = pinocchio::Data(pinocchio_model_);

    // Fetch the index of the frame in the robot model.
    root_frame_index_ = pinocchio_model_.getFrameId(root_frame_name_);
    end_frame_index_ = pinocchio_model_.getFrameId(end_frame_name_);

    // initialize the size of the vectors.
    root_jacobian_.resize(6, pinocchio_model_.nv);
    end_jacobian_.resize(6, pinocchio_model_.nv);
    impedance_jacobian_.resize(6, pinocchio_model_.nv);

    // output
    torques_.resize(pinocchio_model_.nv, 1);
}

void ImpedanceController::run(
    Eigen::Ref<const Eigen::VectorXd> robot_configuration,
    Eigen::Ref<const Eigen::VectorXd> robot_velocity,
    Eigen::Ref<const Array6d> gain_proportional,
    Eigen::Ref<const Array6d> gain_derivative,
    const double& gain_feed_forward_force,
    const pinocchio::SE3& desired_end_frame_placement,
    const pinocchio::Motion& desired_end_frame_velocity,
    const pinocchio::Force& feed_forward_force)
{
    assert(robot_configuration.size() == pinocchio_model_.nq &&
           "robot_configuration is not of the good size.");
    assert(robot_velocity.size() == pinocchio_model_.nv &&
           "robot_velocity is not of the good size.");

    // Get the current frame placements and velocity.
    pinocchio::forwardKinematics(
        pinocchio_model_, pinocchio_data_, robot_configuration, robot_velocity);
    pinocchio::updateFramePlacement(
        pinocchio_model_, pinocchio_data_, root_frame_index_);
    pinocchio::updateFramePlacement(
        pinocchio_model_, pinocchio_data_, end_frame_index_);

    root_placement_ = pinocchio_data_.oMf[root_frame_index_];
    end_placement_ = pinocchio_data_.oMf[end_frame_index_];
    root_velocity_ =
        pinocchio::getFrameVelocity(pinocchio_model_, pinocchio_data_,
                                    root_frame_index_,
                                    pinocchio::LOCAL_WORLD_ALIGNED);
    end_velocity_ = pinocchio::getFrameVelocity(pinocchio_model_,
                                                pinocchio_data_,
                                                end_frame_index_,
                                                pinocchio::LOCAL_WORLD_ALIGNED);

    // Compute the force to be applied to the environment.
    impedance_force_ = gain_proportional * pinocchio::log6(
                           desired_end_frame_placement.actInv(
                               root_placement_.actInv(end_placement_)))
                           .toVector()
                           .array();

    impedance_force_ += (gain_derivative * (desired_end_frame_velocity -
                                            (end_velocity_ - root_velocity_))
                                               .toVector()
                                               .array())
                            .matrix();

    impedance_force_ -=
        (gain_feed_forward_force * feed_forward_force.toVector().array())
            .matrix();

    // Compute the jacobians
    pinocchio::computeJointJacobians(
        pinocchio_model_, pinocchio_data_, robot_configuration);
    pinocchio::getFrameJacobian(pinocchio_model_,
                                pinocchio_data_,
                                end_frame_index_,
                                pinocchio::LOCAL_WORLD_ALIGNED,
                                end_jacobian_);
    pinocchio::getFrameJacobian(pinocchio_model_,
                                pinocchio_data_,
                                root_frame_index_,
                                pinocchio::LOCAL_WORLD_ALIGNED,
                                root_jacobian_);
    impedance_jacobian_ = end_jacobian_ - root_jacobian_;

    // compute the output torques
    torques_ = impedance_jacobian_.transpose() * impedance_force_;
    return;
}

Eigen::VectorXd& ImpedanceController::get_torques()
{
    // model.joint[1].shortname()
    // model.names[1] == "root_joint"
    return torques_;
}

ImpedanceController::Vector6d& ImpedanceController::get_impedance_force()
{
    return impedance_force_;
}

}  // namespace blmc_controllers