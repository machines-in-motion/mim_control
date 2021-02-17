/**
 * @file
 * @license BSD 3-clause
 * @copyright Copyright (c) 2020, New York University and Max Planck
 * Gesellschaft
 *
 * @brief Implements a PD controller at the center of mass.
 *
 */

#include "mim_control/centroidal_force_qp_controller.hpp"

#include <iostream>

namespace mim_control
{
CentroidalForceQPController::CentroidalForceQPController()
{
}

void CentroidalForceQPController::initialize(int number_endeffectors,
                                             double friction_coeff,
                                             Eigen::Ref<const Vector6d> weights)
{
    nb_eff_ = number_endeffectors;
    mu_ = friction_coeff;

    // Resize the problem.
    qp_.reset(3 * nb_eff_ + 6, 9, 5 * nb_eff_);
    sol_.resize(3 * nb_eff_ + 6);
    forces_.resize(3 * nb_eff_);
    sol_.fill(0.);
    forces_.fill(0.);

    // The QP solves the following problem.
    //
    // min. 0.5 * x' Hess x + g0' x
    // s.t. CE x + ce0 = 0
    //      CI x + ci0 >= 0

    hess_.resize(3 * nb_eff_ + 6, 3 * nb_eff_ + 6);
    g0_.resize(3 * nb_eff_ + 6);

    ce_.resize(6, 3 * nb_eff_ + 6);
    ce_new_.resize(6, 3 * nb_eff_ + 6);

    ci_.resize(5 * nb_eff_, 3 * nb_eff_ + 6);
    ci0_.resize(5 * nb_eff_);

    // Simple initializations.
    g0_.fill(0.);
    ci_.fill(0.);
    ci0_.fill(0.);
    ce_.fill(0.);
    hess_.setIdentity();
    hess_ *= 2.;

    // Slack variables weights.
    hess_.block<6, 6>(3 * nb_eff_, 3 * nb_eff_) = weights.asDiagonal();

    // Setup centroidal wrench equality.
    for (int i = 0; i < nb_eff_; i++)
    {
        // Setup the linear part. The angular part with the cross product is
        // setup in the run() method.
        ce_.block<3, 3>(0, 3 * i) << -1, 0, 0, 0, -1, 0, 0, 0, -1;
    }

    // Part of the slack variables.
    ce_.block<6, 6>(0, 3 * nb_eff_).setIdentity();

    // Setup the friction cone constraints.
    for (int j = 0; j < nb_eff_; j++)
    {
        ci_(5 * j + 0, 3 * j + 0) = -1;  // mu Fz - Fx >= 0
        ci_(5 * j + 0, 3 * j + 2) = mu_;
        ci_(5 * j + 1, 3 * j + 0) = 1;  // mu Fz + Fx >= 0
        ci_(5 * j + 1, 3 * j + 2) = mu_;
        ci_(5 * j + 2, 3 * j + 1) = -1;  // mu Fz - Fy >= 0
        ci_(5 * j + 2, 3 * j + 2) = mu_;
        ci_(5 * j + 3, 3 * j + 1) = 1;  // mu Fz + Fy >= 0
        ci_(5 * j + 3, 3 * j + 2) = mu_;
        ci_(5 * j + 4, 3 * j + 2) = 1;  // Fz >= 0
    }
}

void CentroidalForceQPController::run(
    Eigen::Ref<const Vector6d> w_com,
    Eigen::Ref<const Eigen::VectorXd> relative_position_endeff,
    Eigen::Ref<const Eigen::VectorXd> cnt_array)
{
    // Copy the linear part for the centroidal wrench equality.
    ce_new_ = ce_;

    // Setup cross product at the endeffectors.
    for (int i = 0; i < nb_eff_; i++)
    {
        double x = relative_position_endeff(3 * i);
        double y = relative_position_endeff(3 * i + 1);
        double z = relative_position_endeff(3 * i + 2);
        ce_new_.block<3, 3>(3, 3 * i) << 0, z, -y, -z, 0, x, y, -x, 0;
    }

    // Deactivate forces not in contact with the ground.
    for (int i = 0; i < nb_eff_; i++)
    {
        if (cnt_array(i) < 0.2)
        {
            ce_new_.col(3 * i + 0).setZero();
            ce_new_.col(3 * i + 1).setZero();
            ce_new_.col(3 * i + 2).setZero();
        }
    }

    // Solve the QP.
    // std::cout << "Solution result: ";
    // auto status =
    qp_.solve_quadprog(hess_, g0_, ce_new_, w_com, ci_, ci0_, sol_);
    // std::cout << status << std::endl;
    forces_ = sol_.head(3 * nb_eff_);
}

Eigen::VectorXd& CentroidalForceQPController::get_forces()
{
    return forces_;
}

}  // namespace mim_control
