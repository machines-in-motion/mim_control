/**
 * @file
 * @license BSD 3-clause
 * @copyright Copyright (c) 2020, New York University and Max Planck
 * Gesellschaft
 *
 * @brief Implementation of the ImpedanceController3D class.
 */

#include "mim_control/impedance_controller3d.hpp"
#include "pinocchio/algorithm/frames.hpp"

namespace mim_control
{
ImpedanceController3D::ImpedanceController3D()
{
}

void ImpedanceController3D::initialize(const pinocchio::Model& pinocchio_model,
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
    root_jacobian_.fill(0.);
    end_jacobian_.resize(6, pinocchio_model_.nv);
    end_jacobian_.fill(0.);
    impedance_jacobian_.resize(6, pinocchio_model_.nv);
    impedance_jacobian_.fill(0.);

    // Defines if the model has a freeflyer.
    pinocchio_model_has_free_flyer_ =
        pinocchio_model_.joints[1].shortname() == "JointModelFreeFlyer";

    // output
    torques_.resize(pinocchio_model_.nv, 1);
    torques_.fill(0.);
    if (pinocchio_model_has_free_flyer_)
        joint_torques_.resize(pinocchio_model_.nv, 1);
    else
    {
        joint_torques_.resize(pinocchio_model_.nv - 6, 1);
    }
}

void ImpedanceController3D::run(
    Eigen::Ref<const Eigen::VectorXd> robot_configuration,
    Eigen::Ref<const Eigen::VectorXd> robot_velocity,
    Eigen::Ref<const Eigen::Array3d> gain_proportional,
    Eigen::Ref<const Eigen::Array3d> gain_derivative,
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
        pinocchio::getFrameVelocity(pinocchio_model_,
                                    pinocchio_data_,
                                    root_frame_index_,
                                    pinocchio::LOCAL_WORLD_ALIGNED);
    end_velocity_ = pinocchio::getFrameVelocity(pinocchio_model_,
                                                pinocchio_data_,
                                                end_frame_index_,
                                                pinocchio::LOCAL_WORLD_ALIGNED);

    // End position and velocity relative to the root.
    root_p_end_ = end_placement_.translation() - root_placement_.translation();
    root_v_end_ = end_velocity_.linear() - root_velocity_.linear();

    // Errors computation.
    position_error_ = desired_end_frame_placement.translation() - root_p_end_;
    velocity_error_ = desired_end_frame_velocity.linear() - root_v_end_;

    // Compute the force to be applied to the environment.
    impedance_force_ = -gain_proportional.head(3) * position_error_.array();
    impedance_force_ -= (gain_derivative * velocity_error_.array()).matrix();

    impedance_force_ +=
        gain_feed_forward_force * feed_forward_force.linear();

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
    impedance_jacobian_ = end_jacobian_;  // - root_jacobian_;

    // compute the output torques
    torques_ = - impedance_jacobian_.topRows(3).transpose() * impedance_force_;

    if (pinocchio_model_has_free_flyer_)
        joint_torques_ = torques_.tail(pinocchio_model_.nv - 6);
    else
    {
        joint_torques_ = torques_;
    }
    return;
}

const Eigen::VectorXd& ImpedanceController3D::get_torques()
{
    return torques_;
}

const Eigen::VectorXd& ImpedanceController3D::get_joint_torques()
{
    return joint_torques_;
}

const Eigen::Vector3d& ImpedanceController3D::get_impedance_force()
{
    return impedance_force_;
}

}  // namespace mim_control