#include "pinocchio/spatial/se3.hpp"
#include "pinocchio/spatial/skew.hpp"
#include <Eigen/Dense>
#include <iostream>

/**
 * @brief Compute the unit vector of a vector
*/
Eigen::Vector3d unit(const Eigen::Vector3d &v)
{
    double n = v.norm();
    if (n < std::numeric_limits<double>::epsilon())
    {
        throw std::runtime_error("vector has zero norm");
    }

    return v / n;
}

/**
 * @brief Compute rotation matrix for a given DH parameters of one joint
*/
Eigen::Matrix3d dh_R(double alpha, double theta)
{
    Eigen::Matrix3d T;
    T << cos(theta), -sin(theta) * cos(alpha), sin(theta) * sin(alpha), sin(theta), cos(theta) * cos(alpha),
        -cos(theta) * sin(alpha), 0.0, sin(alpha), cos(alpha);

    return T;
}


Eigen::Ref<Eigen::Matrix<double, 7, 1>> iiwa7_ik(const pinocchio::SE3 &pose,
                                                 const double psi,
                                                 Eigen::Ref<Eigen::Matrix<double, 7, 1>> q)
{
    double lBS = 0.34;
    double lSE = 0.4;
    double lEW = 0.4;
    double lWT = 0.126;

    // configuration parameters
    double arm = 1.0;
    double elbow = 1.0;
    double wrist = 1.0;

    // end-effector position from base
    const Eigen::Vector3d xend = pose.translation();
    // shoulder position from base
    const Eigen::Vector3d xs = Eigen::Vector3d(0, 0, lBS);
    // end-effector position from wrist
    const Eigen::Vector3d xwt = Eigen::Vector3d(0, 0, lWT);
    // wrist position from base
    const Eigen::Vector3d xw = xend - pose.rotation() * xwt;
    // shoulder to wrist vector
    const Eigen::Vector3d xsw = xw - xs;
    const Eigen::Vector3d usw = unit(xsw);
    double dsw = xsw.norm();

    // Check if pose is within arm+forearm reach
    assert((dsw < lSE + lEW) && (dsw > lSE - lEW) && "Specified pose outside reachable workspace");

    // Joint 4 calculation
    // Elbow joint can be directly calculated since it does only depend on the robot configuration and the xsw vector
    q(3) = elbow * std::acos((dsw * dsw - lSE * lSE - lEW * lEW) / (2 * lSE * lEW));

    Eigen::Matrix3d R34 = dh_R(-M_PI / 2.0, q(3));

    double q0_o = 0.0;
    if (xsw.cross(Eigen::Vector3d(0, 0, 1)).norm() > 1e-6)
    {
        q0_o = atan2(xsw(1), xsw(0));
    }

    const double phi = acos((lSE * lSE + dsw * dsw - lEW * lEW) / (2 * lSE * dsw));
    double q1_o = atan2(std::hypot(xsw(0), xsw(1)), xsw(2)) + elbow * phi;

    Eigen::Matrix3d R03_o = dh_R(-M_PI / 2.0, q0_o) * dh_R(M_PI / 2.0, q1_o) * dh_R(M_PI / 2.0, 0);

    Eigen::Matrix3d skew_usw = Eigen::Matrix3d::Zero();
    pinocchio::skew(usw, skew_usw);

    Eigen::Matrix3d As = skew_usw * R03_o;
    Eigen::Matrix3d Bs = -skew_usw * skew_usw * R03_o;
    Eigen::Matrix3d Cs = skew_usw * skew_usw.transpose() * R03_o;

    Eigen::Matrix3d R03 = As * sin(psi) + Bs * cos(psi) + Cs;

    q(0) = atan2(arm * R03(1, 1), arm * R03(0, 1));
    q(1) = arm * acos(R03(2, 1));
    q(2) = atan2(arm * R03(2, 2), -arm * R03(2, 0));

    Eigen::Matrix3d R47 =
        R34.transpose() * (As.transpose() * sin(psi) + Bs.transpose() * cos(psi) + Cs.transpose()) * pose.rotation();

    q(4) = atan2(wrist * R47(1, 2), wrist * R47(1, 2));
    q(5) = wrist * acos(R47(2, 2));
    q(6) = atan2(wrist * R47(2, 1), -wrist * R47(2, 0));

    return q;
}

int main()
{
    pinocchio::SE3 T = pinocchio::SE3::Identity();
    T.translation() = Eigen::Vector3d(0.5, 0.5, 0.5);
    Eigen::Matrix<double, 7, 1> q = Eigen::Matrix<double, 7, 1>::Zero();
    iiwa7_ik(T, M_PI / 2, q);

    std::cout << "T = \n" << T << std::endl;
    std::cout << "q = \n" << q.transpose() << std::endl;
    return 0;
}