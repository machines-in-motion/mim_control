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

#include "pinocchio/multibody/data.hpp"
#include "pinocchio/multibody/model.hpp"

#include "mim_control/centroidal_force_qp_controller.hpp"
#include "mim_control/centroidal_pd_controller.hpp"
#include "mim_control/impedance_controller.hpp"


namespace mim_control
{
/**
 * @brief Controller for running centroidal pd, force qp and impedance at once.
 */
class CentroidalImpedanceController
{
public:
    typedef Eigen::Array<double, 6, 1> Array6d;
    typedef Eigen::Matrix<double, 6, 1> Vector6d;

    /**
     * @brief Construct a new CentroidalImpedanceController object.
     */
    CentroidalImpedanceController();

    /**
     * @brief Initialize the internal data. None real-time safe method.
     *
     * @param pinocchio_model rigid body model of the robot
     * @param root_frame_name root frame name where the spring starts(Ex. Hip)
     * @param end_frame_name frame name where the spring ends(Ex. end effector)
     */
    void initialize(const double& mass,
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
                    Eigen::Ref<const Array6d> frame_velocity_error_gain);

    void update_centroidal_gains(Eigen::Ref<const Eigen::Vector3d> kc,
                                 Eigen::Ref<const Eigen::Vector3d> dc,
                                 Eigen::Ref<const Eigen::Vector3d> kb,
                                 Eigen::Ref<const Eigen::Vector3d> db);

    void update_endeff_gains(Eigen::Ref<const Array6d> frame_placement_error_gain,
                             Eigen::Ref<const Array6d> frame_velocity_error_gain);

    void run(Eigen::Ref<const Eigen::VectorXd> robot_configuration,
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
             Eigen::Ref<const Eigen::VectorXd> desired_end_frame_velocity);

    /**
     * @brief Get the computed joint torques from the impedance controller.
     *
     * @return Eigen::VectorXd&
     */
    const Eigen::VectorXd& get_joint_torques();

private:
    /** @brief Rigid body dynamics model. */
    pinocchio::Model pinocchio_model_;

    /** @brief Cache of the rigid body dynamics algorithms. */
    pinocchio::Data pinocchio_data_;

    double mass_;
    Eigen::Vector3d kc_;
    Eigen::Vector3d dc_;
    Eigen::Vector3d kb_;
    Eigen::Vector3d db_;
    Array6d frame_placement_error_gain_;
    Array6d frame_velocity_error_gain_;

    Eigen::VectorXd joint_torques_;
    Eigen::VectorXd relative_position_endeff_;

    CentroidalPDController centroidal_pd_controller_;
    CentroidalForceQPController centroidal_force_qp_controller_;
    std::vector<ImpedanceController> impedance_controllers_;
};

}  // namespace mim_control
