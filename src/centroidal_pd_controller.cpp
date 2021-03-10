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
#include "pinocchio/spatial/explog.hpp"

namespace mim_control
{
CentroidalPDController::CentroidalPDController()
{
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
    des_ori_quat_.w() = ori_des[3];
    des_ori_quat_.vec()[0] = ori_des[0];
    des_ori_quat_.vec()[1] = ori_des[1];
    des_ori_quat_.vec()[2] = ori_des[2];

    ori_quat_.w() = ori[3];
    ori_quat_.vec()[0] = ori[0];
    ori_quat_.vec()[1] = ori[1];
    ori_quat_.vec()[2] = ori[2];

    des_ori_se3_ = des_ori_quat_.toRotationMatrix();
    ori_se3_ = ori_quat_.toRotationMatrix();

    /*************************************************************************/
    // Compute the linear part of the wrench.

    /*---------- computing position error ----*/
    pos_error_.array() = com_des.array() - com.array();
    vel_error_.array() = vcom_des.array() - vcom.array();
    /*---------- computing tourques ----*/

    wrench_.head<3>().array() = mass_ * (pos_error_.array() * kc.array() +
                                         vel_error_.array() * dc.array());

    /*************************************************************************/
    // Compute the angular part of the wrench.
    ori_error_se3_ = des_ori_se3_.transpose() * ori_se3_;
    ori_error_quat_ = ori_error_se3_;

    ori_error_ = pinocchio::quaternion::log3(des_ori_quat_ * ori_quat_.conjugate());

    /*---------- computing ang error ----*/

    // Rotate the des and current angular velocity into the world frame.
    angvel_world_error_ = ori_se3_ * angvel;
    des_angvel_world_error_ = des_ori_se3_ * angvel_des;

    wrench_.tail<3>().array() =
        db.array() * inertia_.array() * (
            des_angvel_world_error_.array() - angvel_world_error_.array()
        ) +
        kb.array() * ori_error_.array();
}

Vector6d& CentroidalPDController::get_wrench()
{
    return wrench_;
}

}  // namespace mim_control
