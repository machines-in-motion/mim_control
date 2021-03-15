/**
 * @file
 * @license BSD 3-clause
 * @copyright Copyright (c) 2020, New York University and Max Planck
 * Gesellschaft
 *
 * @brief Implements a PD controller at the center of mass.
 *
 */

#pragma once

#include <Eigen/Dense>

namespace mim_control
{
typedef Eigen::Array<double, 6, 1> Array6d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;

/**
 * @brief Impedance controller between any two frames of the robot.
 */
class CentroidalPDController
{
public:
    /**
     * @brief Construct a new ImpedanceController object.
     */
    CentroidalPDController();

    /**
     * @brief Initialize the internal data. None real-time safe method.
     *
     * @param inertia Inertia of the base.
     */
    void initialize(const double& mass,
                    Eigen::Ref<const Eigen::Vector3d> inertia);

    /**
     * Computes the centroidal wrench using a PD controller.
     */
    void run(Eigen::Ref<const Eigen::Vector3d> kc,
             Eigen::Ref<const Eigen::Vector3d> dc,
             Eigen::Ref<const Eigen::Vector3d> kb,
             Eigen::Ref<const Eigen::Vector3d> db,
             Eigen::Ref<const Eigen::Vector3d> com,
             Eigen::Ref<const Eigen::Vector3d> com_des,
             Eigen::Ref<const Eigen::Vector3d> vcom,
             Eigen::Ref<const Eigen::Vector3d> vcom_des,
             Eigen::Ref<const Eigen::Vector4d> ori,
             Eigen::Ref<const Eigen::Vector4d> ori_des,
             Eigen::Ref<const Eigen::Vector3d> angvel,
             Eigen::Ref<const Eigen::Vector3d> angvel_des);

    /**
     * @brief Get the computed wrench from the PD controller.
     *
     * @return Eigen::VectorXd&
     */
    Vector6d& get_wrench();

private:  // attributes
    /** @brief Output wrench */
    Vector6d wrench_;

    double mass_;
    Eigen::Vector3d inertia_;

    Eigen::Vector3d pos_error_;
    Eigen::Vector3d vel_error_;
    Eigen::Vector3d ori_error_;

    Eigen::Vector3d angvel_world_error_;
    Eigen::Vector3d des_angvel_world_error_;

    Eigen::Quaternion<double> ori_quat_;
    Eigen::Quaternion<double> des_ori_quat_;
    Eigen::Quaternion<double> ori_error_quat_;

    Eigen::Matrix<double, 3, 3> ori_se3_;
    Eigen::Matrix<double, 3, 3> des_ori_se3_;
    Eigen::Matrix<double, 3, 3>
        ori_error_se3_;  // refer to christian ott paper for definitions (Rdb)
};

}  // namespace mim_control
