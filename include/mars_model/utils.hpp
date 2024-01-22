#ifndef MARS_UTILS_HPP
#define MARS_UTILS_HPP

#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/model.hpp"
#include "pinocchio/parsers/urdf.hpp"

#include <string>
#include <vector>

namespace mars
{
/**
 * @brief transformation matrix from twist (in world frame) to quaternion time derivative
 *
 * dq = Z(quat)*w
 *
 * @param quat
 * @param Z
 */
template <typename T>
void OMEGA(const Eigen::Quaternion<T> &quat, Eigen::Matrix<T, 4, 3> &Z)
{
    Z.row(0) << quat.w(), quat.z(), -quat.y();   // dq_x
    Z.row(1) << -quat.z(), quat.w(), quat.x();   // dq_y
    Z.row(2) << quat.y(), -quat.x(), quat.w();   // dq_z
    Z.row(3) << -quat.x(), -quat.y(), -quat.z(); // dq_w

    Z = 0.5 * Z;
}

/**
 * @brief Transform a twist from frame A to frame B, Note that r_AB and x_A must be expressed in the same frame
*/
pinocchio::MotionTpl<double, 0> &transform_twist(const Eigen::Vector3d &r_AB,
                                                 const pinocchio::MotionTpl<double, 0> &x_A,
                                                 pinocchio::MotionTpl<double, 0> &x_B)
{
    // v_B = v_A + w_A x (r_A - r_B)
    x_B.linear(x_A.linear() + (x_A.angular()).cross(r_AB));
    // w_B = w_A
    x_B.angular(x_A.angular());

    return x_B;
}


/**
 * @brief Transform a twist derivative from frame A to frame B, Note that r_AB and x_A must be expressed in the same frame
*/
pinocchio::MotionTpl<double, 0> &transform_twist_derivative(const Eigen::Vector3d &r_AB,
                                                            const pinocchio::MotionTpl<double, 0> &x_A,
                                                            const pinocchio::MotionTpl<double, 0> &dx_A,
                                                            pinocchio::MotionTpl<double, 0> &dx_B)
{
    // dv_B = dv_A + dw_A x (r_A - r_B) + w_A x w_A x (r_A - r_B)
    dx_B.linear(dx_A.linear() + (dx_A.angular()).cross(r_AB) + (x_A.angular()).cross(x_A.angular().cross(r_AB)));
    // dw_B = dw_A
    dx_B.angular(dx_A.angular());

    return dx_B;
}

} // namespace mars

#endif // MARS_UTILS_HPP
