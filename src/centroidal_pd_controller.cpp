/**
 * @file
 * @license BSD 3-clause
 * @copyright Copyright (c) 2020, New York University and Max Planck
 * Gesellschaft
 *
 * @brief Implements a PD controller at the center of mass.
 *
 */

#include "mim_control/centroidal_pd_controller.hpp"
#include "pinocchio/math/rpy.hpp"
#include "pinocchio/spatial/explog.hpp"

namespace mim_control
{
CentroidalPDController::CentroidalPDController()
{
    wrench_.setZero();
    wrench_local_.setZero();

    mass_ = 0.0;
    inertia_.setZero();

    pos_error_.setZero();
    vel_error_.setZero();
    ori_error_.setZero();
    angvel_error_.setZero();

    ori_quat_.setIdentity();
    des_ori_quat_.setIdentity();

    ori_rot_mat_.setIdentity();
    des_ori_rot_mat_.setIdentity();
    ori_yaw_.setZero();

    world_R_local_.setIdentity();
}

void CentroidalPDController::initialize(
    const double& mass, Eigen::Ref<const Eigen::Vector3d> inertia)
{
    mass_ = mass;
    inertia_ = inertia;
}

void CentroidalPDController::run(Eigen::Ref<const Eigen::Vector3d> kc,
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
                                 Eigen::Ref<const Eigen::Vector3d> angvel_des)
{
    /*
     * Convert inputs.
     */
    // Normalize the quaternion
    des_ori_quat_.x() = ori_des[0];
    des_ori_quat_.y() = ori_des[1];
    des_ori_quat_.z() = ori_des[2];
    des_ori_quat_.w() = ori_des[3];
    des_ori_quat_.normalize();
    // idem.
    ori_quat_.x() = ori[0];
    ori_quat_.y() = ori[1];
    ori_quat_.z() = ori[2];
    ori_quat_.w() = ori[3];
    ori_quat_.normalize();

    // Convert to rotation matrix.
    des_ori_rot_mat_ = des_ori_quat_.toRotationMatrix();
    ori_rot_mat_ = ori_quat_.toRotationMatrix();

    /*
     * Create the control frame.
     */
    ori_yaw_ = pinocchio::rpy::matrixToRpy(ori_rot_mat_);
    ori_yaw_.head<2>().setZero();
    world_R_local_ = pinocchio::rpy::rpyToMatrix(ori_yaw_);
    world_R_local_.setIdentity();

    /*
     * Compute the linear part of the wrench.
     */
    // Compute linear error.
    pos_error_.array() = /*world_R_local_.transpose() * */ (com_des - com);
    vel_error_.array() = /*world_R_local_.transpose() * */ (vcom_des - vcom);
    // Compute linear wrench.
    wrench_local_.head<3>().array() = mass_ * (kc.array() * pos_error_.array() +
                                         dc.array() * vel_error_.array());

    /*
     * Compute the angular part of the wrench.
     */
    // Compute angular error.
    ori_error_ =
        /*world_R_local_.transpose() * */
        pinocchio::quaternion::log3(des_ori_quat_ * ori_quat_.conjugate());
    angvel_error_ = /*world_R_local_.transpose() * */
                    ((des_ori_rot_mat_ * angvel_des) - (ori_rot_mat_ * angvel));
    // Compute amgular wrench.
    wrench_local_.tail<3>().array() =
        kb.array() * ori_error_.array() +
        db.array() * inertia_.array() * angvel_error_.array();

    /*
     * Convert the wrench back into the world frame.
     */
    wrench_.head<3>() = /* world_R_local_ * */ wrench_local_.head<3>();
    wrench_.tail<3>() = /* world_R_local_ * */ wrench_local_.tail<3>();
}

Vector6d& CentroidalPDController::get_wrench()
{
    return wrench_;
}

}  // namespace mim_control
