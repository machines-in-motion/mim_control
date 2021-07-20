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

namespace mim_control
{
/**
 * @brief Impedance controller between any two frames of the robot.
 */
class ImpedanceController
{
public:
    typedef Eigen::Array<double, 6, 1> Array6d;
    typedef Eigen::Matrix<double, 6, 1> Vector6d;

    /**
     * @brief Construct a new ImpedanceController object.
     */
    ImpedanceController();

    /**
     * @brief Initialize the internal data. None real-time safe method.
     *
     * @param pinocchio_model rigid body model of the robot
     * @param root_frame_name root frame name where the spring starts(Ex. Hip)
     * @param end_frame_name frame name where the spring ends(Ex. end effector)
     */
    void initialize(const pinocchio::Model& pinocchio_model,
                    const std::string& root_frame_name,
                    const std::string& end_frame_name);

    /**
     * @brief Computes the desired joint torques
     *
     * \f$
     * \tau = J^T (k_p (x^{des} - x) +
     *        k_d (\dot{x}^{des} - \dot{x}) -k_f f)
     * \f$
     *
     * with:
     * - \f$ \tau \f$ the joint torques,
     * - \f$ J \f$ the Jacobian of the sub-kinematic-tree between the root and
     *   the end frame,
     * - \f$ k_p \f$ the gain proportional to the frame placement error,
     * - \f$ x_{des} \f$ desired end frame placement with respect to the
     *   desired root frame.
     * - \f$ x \f$ measured end frame placement with respect to the
     *   measured root frame.
     * - \f$ k_d \f$ is the derivative gain applied to the time derivative of
     *   the error.
     * - \f$ \dot{x}^{des} \f$ desired end frame velocity with respect to the
     *   desired root frame.
     * - \f$ \dot{x} \f$ measured end frame velocity with respect to the
     *   measured root frame.
     * - \f$ k_f \f$ the gain over the feed forward force,
     * - \f$ f \f$ the feed forward force,
     *
     * @param robot_configuration robot generalized coordinates configuration.
     * @param robot_velocity robot generalized coordinates velocity.
     * @param gain_proportional 6d vector for the proportional gains on {x, y,
     * z, roll, pitch, yaw}.
     * @param gain_derivative 6d vector for the proportional gains on {x, y, z,
     * roll, pitch, yaw}.
     * @param gain_feed_forward_force gain multiplying the feed forward force.
     * @param desired_end_frame_placement desired end frame placement relative
     * to the desired root joint.
     * @param desired_end_frame_velocity desired end frame velocity relative to
     * the desired root joint.
     * @param feed_forward_force feed forward force applied to the foot by the
     * environment.
     */
    void run(Eigen::Ref<const Eigen::VectorXd> robot_configuration,
             Eigen::Ref<const Eigen::VectorXd> robot_velocity,
             Eigen::Ref<const Array6d> gain_proportional,
             Eigen::Ref<const Array6d> gain_derivative,
             const double& gain_feed_forward_force,
             const pinocchio::SE3& desired_end_frame_placement,
             const pinocchio::Motion& desired_end_frame_velocity,
             const pinocchio::Force& feed_forward_force);


    /**
     * @brief Similar to `run()` but with the data already precomputed.
     *
     * @param pinocchio_data The data object to use for the computation.
     * @param gain_proportional 6d vector for the proportional gains on {x, y,
     * z, roll, pitch, yaw}.
     * @param gain_derivative 6d vector for the proportional gains on {x, y, z,
     * roll, pitch, yaw}.
     * @param gain_feed_forward_force gain multiplying the feed forward force.
     * @param desired_end_frame_placement desired end frame placement relative
     * to the desired root joint.
     * @param desired_end_frame_velocity desired end frame velocity relative to
     * the desired root joint.
     * @param feed_forward_force feed forward force applied to the foot by the
     * environment.
     */
    void run_precomputed_data(pinocchio::Data& pinocchio_data,
                              Eigen::Ref<const Array6d> gain_proportional,
                              Eigen::Ref<const Array6d> gain_derivative,
                              const double& gain_feed_forward_force,
                              const pinocchio::SE3& desired_end_frame_placement,
                              const pinocchio::Motion& desired_end_frame_velocity,
                              const pinocchio::Force& feed_forward_force);

    /**
     * @brief Get the computed torques from the impedance controller.
     *
     * @return Eigen::VectorXd&
     */
    const Eigen::VectorXd& get_torques();

    /**
     * @brief Get the computed joint torques from the impedance controller.
     *
     * @return Eigen::VectorXd&
     */
    const Eigen::VectorXd& get_joint_torques();

    /**
     * @brief Get the impedance force \f$ f_i \f$ with \f$ \tau = J^T f_i \f$.
     *
     * @return Vector6d&
     */
    const Vector6d& get_impedance_force();

    /**
     * @brief Returns the index of the endeffector frame.
     *
     * @return pinocchio::FrameIndex
     */
    const pinocchio::FrameIndex& get_endframe_index();

private:  // attributes
    /** @brief Rigid body dynamics model. */
    pinocchio::Model pinocchio_model_;

    /** @brief Cache of the rigid body dynamics algorithms. */
    pinocchio::Data pinocchio_data_;

    /** @brief (urdf) name of the root frame. The impedance controller is
     * computed with respect to this frame. */
    std::string root_frame_name_;

    /** @brief Index of the root frame in the pinocchio model. */
    pinocchio::FrameIndex root_frame_index_;

    /** @brief Measured root frame placement. */
    pinocchio::SE3 root_placement_;

    /** @brief Measured root frame orientation. */
    pinocchio::SE3 root_orientation_;

    /** @brief Measured root frame velocity. */
    pinocchio::Motion root_velocity_;

    /** @brief (urdf) name of the end frame. This is the controlled frame. */
    std::string end_frame_name_;

    /** @brief Index of the end frame in the pinocchio model. */
    pinocchio::FrameIndex end_frame_index_;

    /** @brief Jacobian of the end frame. */
    pinocchio::Data::Matrix6x end_jacobian_;

    /** @brief Measured end frame placement. */
    pinocchio::SE3 end_placement_;

    /** @brief Measured end frame orientation. */
    pinocchio::SE3 end_orientation_;

    /** @brief Measured end frame velocity. */
    pinocchio::Motion end_velocity_;

    /** @brief Impedance force. Accessible for machine learning purposes. */
    Vector6d impedance_force_;

    /** @brief Measured end frame placement in root frame. */
    pinocchio::SE3 actual_end_frame_placement_;

    /** @brief Measured end frame placement in root frame. */
    pinocchio::Motion actual_end_frame_velocity_;

    /** @brief SE3 placement error. */
    Eigen::Matrix<double, 6, 1> err_se3_;

    /** @brief SE3 velocity error. */
    pinocchio::Motion err_vel_;

    /** @brief Jacobian used in the computation of the impedance. */
    pinocchio::Data::Matrix6x impedance_jacobian_;

    /** @brief Output torques. */
    Eigen::VectorXd torques_;

    /** @brief Output torques. */
    Eigen::VectorXd joint_torques_;

    /** @brief Checks out if the Pinocchio rigid body model of the robot
     * contains a free-flyer. This is used to return the command i.e. the
     * torques to be applied to the joints only. */
    bool pinocchio_model_has_free_flyer_;
};

}  // namespace mim_control
