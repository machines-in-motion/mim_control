/**
 * @file
 * @license BSD 3-clause
 * @copyright Copyright (c) 2021, New York University and Max Planck
 * Gesellschaft
 *
 * @brief Implements the CentroidalImpedance controller.
 */

#include "mim_control/centroidal_impedance_controller.hpp"

#include "pinocchio/algorithm/frames.hpp"

namespace mim_control
{
CentroidalImpedanceController::CentroidalImpedanceController()
{
}

void CentroidalImpedanceController::initialize(
    const double& mass,
    Eigen::Ref<const Eigen::Vector3d> inertia,
    const pinocchio::Model& pinocchio_model,
    const std::string& root_frame_name,
    const std::vector<std::string>& end_frame_names,
    double friction_coeff,
    Eigen::Ref<const Vector6d> qp_penalty_weights,
    Eigen::Ref<const Eigen::Vector3d> kc,
    Eigen::Ref<const Eigen::Vector3d> dc,
    Eigen::Ref<const Eigen::Vector3d> kb,
    Eigen::Ref<const Eigen::Vector3d> db,
    Eigen::Ref<const Array6d> frame_placement_error_gain,
    Eigen::Ref<const Array6d> frame_velocity_error_gain
)
{
    bool is_free_flyer =
        pinocchio_model.joints[1].shortname() == "JointModelFreeFlyer";

    mass_ = mass; 
    kc_ = kc;
    dc_ = dc;
    kb_ = kb;
    db_ = db;
    frame_placement_error_gain_ = frame_placement_error_gain;
    frame_velocity_error_gain_ = frame_velocity_error_gain;

    pinocchio_model_ = pinocchio_model;

    if (is_free_flyer) {
        joint_torques_.resize(pinocchio_model.nv - 6);
    } else {
        joint_torques_.resize(pinocchio_model.nv);
    }

    // Create the cache of the rigid body dynamics algorithms
    pinocchio_data_ = pinocchio::Data(pinocchio_model_);

    int num_endeff = end_frame_names.size();

    relative_position_endeff_.resize(3 * num_endeff);

    // Initialize the other controllers.
    centroidal_pd_controller_.initialize(mass, inertia);

    centroidal_force_qp_controller_.initialize(
        num_endeff,
        friction_coeff,
        qp_penalty_weights
    );

    impedance_controllers_.resize(num_endeff);
    for (int i = 0; i < num_endeff; i++)
    {
        impedance_controllers_[i].initialize(
            pinocchio_model_,
            root_frame_name, end_frame_names[i]);
    }
}

void CentroidalImpedanceController::update_centroidal_gains(
                            Eigen::Ref<const Eigen::Vector3d> kc,
                            Eigen::Ref<const Eigen::Vector3d> dc,
                            Eigen::Ref<const Eigen::Vector3d> kb,
                            Eigen::Ref<const Eigen::Vector3d> db)
{
    kc_ = kc;
    dc_ = dc;
    kb_ = kb;
    db_ = db;
}

void CentroidalImpedanceController::update_endeff_gains(
                        Eigen::Ref<const Array6d> frame_placement_error_gain,
                        Eigen::Ref<const Array6d> frame_velocity_error_gain)
{
    frame_placement_error_gain_ = frame_placement_error_gain;
    frame_velocity_error_gain_ = frame_velocity_error_gain;
}

void CentroidalImpedanceController::run(
    Eigen::Ref<const Eigen::VectorXd> robot_configuration,
    Eigen::Ref<const Eigen::VectorXd> robot_velocity,
    Eigen::Ref<const Eigen::VectorXd> cnt_array,
    Eigen::Ref<const Eigen::Vector3d> com,
    Eigen::Ref<const Eigen::Vector3d> com_des,
    Eigen::Ref<const Eigen::Vector3d> vcom,
    Eigen::Ref<const Eigen::Vector3d> vcom_des,
    Eigen::Ref<const Eigen::Vector4d> ori,
    Eigen::Ref<const Eigen::Vector4d> ori_des,
    Eigen::Ref<const Eigen::Vector3d> angvel,
    Eigen::Ref<const Eigen::Vector3d> angvel_des,
    Eigen::Ref<const Eigen::VectorXd> desired_end_frame_placement,
    Eigen::Ref<const Eigen::VectorXd> desired_end_frame_velocity
)
{
    //////
    // Compute the desired centroidal wrench.
    centroidal_pd_controller_.run(
        kc_, dc_, kb_, db_, com, com_des, vcom, vcom_des, ori, ori_des,
        angvel, angvel_des);

    Vector6d& w_com = centroidal_pd_controller_.get_wrench();
    w_com(2) += 9.81*mass_; 
    //////
    // Compute the forces at the endeffectors.

    // Compute the relative positions of the endffectors.
    int num_endeff = impedance_controllers_.size();
    for (int i = 0; i < num_endeff; i++)
    {
        auto idx = impedance_controllers_[i].get_endframe_index();
        relative_position_endeff_.block<3, 1>(3 * i, 0) = (
            pinocchio_data_.oMf[idx].translation() - com
        );
    }

    centroidal_force_qp_controller_.run(
        w_com, relative_position_endeff_, cnt_array);

    Eigen::VectorXd& forces = centroidal_force_qp_controller_.get_forces();

    //////
    // Update the pinocchio data.
    pinocchio::forwardKinematics(
        pinocchio_model_, pinocchio_data_, robot_configuration, robot_velocity);
    pinocchio::updateFramePlacements(
        pinocchio_model_, pinocchio_data_);
    pinocchio::computeJointJacobians(
        pinocchio_model_, pinocchio_data_, robot_configuration);

    //////
    // Compute the joint torques.
    joint_torques_.fill(0);

    for (int i = 0; i < num_endeff; i++)
    {
        pinocchio::SE3::Quaternion q (
            desired_end_frame_placement[i * 7 + 6],
            desired_end_frame_placement[i * 7 + 3],
            desired_end_frame_placement[i * 7 + 4],
            desired_end_frame_placement[i * 7 + 5]);
        pinocchio::SE3::Vector3 t (
            desired_end_frame_placement[i * 7 + 0],
            desired_end_frame_placement[i * 7 + 1],
            desired_end_frame_placement[i * 7 + 2]);

        pinocchio::SE3 pos(q.matrix(), t);
        pinocchio::Motion vel(desired_end_frame_velocity.block<6, 1>(i * 6, 0));

        Vector6d f;
        f.fill(0);
        f.head<3>() = forces.block<3, 1>(3 * i, 0);
        pinocchio::Force F(f);

        impedance_controllers_[i].run_precomputed_data(
            pinocchio_data_,
            frame_placement_error_gain_, frame_velocity_error_gain_,
            1.,
            pos, vel, F
        );

        joint_torques_ += impedance_controllers_[i].get_joint_torques();
    }
}

const Eigen::VectorXd& CentroidalImpedanceController::get_joint_torques()
{
    return joint_torques_;
}

}
