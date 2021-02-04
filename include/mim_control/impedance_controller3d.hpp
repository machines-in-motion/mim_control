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
 * @brief Impedance controller between any two points of the robot.
 */
class ImpedanceController3D
{
public:
    typedef Eigen::Array<double, 6, 1> Array6d;
    typedef Eigen::Matrix<double, 6, 1> Vector6d;

    /**
     * @brief Construct a new ImpedanceController3D object.
     */
    ImpedanceController3D();

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
     * - \f$ k_p \f$ the gain proportional to the 3D position error,
     * - \f$ x_{des} \f$ desired end frame 3D position with respect to the
     *   desired root frame.
     * - \f$ x \f$ measured end 3D position with respect to the
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
     * @param desired_end_frame_placement desired end 3D position relative
     * to the desired root frame.
     * @param desired_end_frame_velocity desired end frame velocity relative to
     * the desired root frame.
     * @param feed_forward_force feed forward force applied to the foot by the
     * environment.
     */
    void run(Eigen::Ref<const Eigen::VectorXd> robot_configuration,
             Eigen::Ref<const Eigen::VectorXd> robot_velocity,
             Eigen::Ref<const Eigen::Array3d> gain_proportional,
             Eigen::Ref<const Eigen::Array3d> gain_derivative,
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
     * @return Vector3d&
     */
    const Eigen::Vector3d& get_impedance_force();

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

    /** @brief Jacobian of the root frame. */
    pinocchio::Data::Matrix6x root_jacobian_;

    /** @brief Measured root 3D position. */
    pinocchio::SE3 root_placement_;

    /** @brief Measured root frame velocity. */
    pinocchio::Motion root_velocity_;

    /** @brief (urdf) name of the end frame. This is the controlled frame. */
    std::string end_frame_name_;

    /** @brief Index of the end frame in the pinocchio model. */
    pinocchio::FrameIndex end_frame_index_;

    /** @brief Jacobian of the end frame. */
    pinocchio::Data::Matrix6x end_jacobian_;

    /** @brief Measured end 3D position. */
    pinocchio::SE3 end_placement_;

    /** @brief Measured end frame velocity. */
    pinocchio::Motion end_velocity_;

    /** @brief Measured relative position between root and end. */
    Eigen::Vector3d root_p_end_;

    /** @brief Measured relative velocity between root and end. */
    Eigen::Vector3d root_v_end_;

    /** @brief position error for the PD controller. */
    Eigen::Vector3d position_error_;

    /** @brief velocity error for the PD controller. */
    Eigen::Vector3d velocity_error_;

    /** @brief Impedance force. Accessible for machine learning purposes. */
    Eigen::Vector3d impedance_force_;

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
