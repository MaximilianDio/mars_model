#include <iostream>

#include "eigen3/Eigen/Dense"
#include "mars_model/se3_model.hpp"

#include "gtest/gtest.h"

// PINOCCHIO_MODEL_DIR is defined by the CMake but you can define your own directory here.
#ifndef PINOCCHIO_MODEL_DIR
#define PINOCCHIO_MODEL_DIR "/workspace/models/object_3d"
#endif

TEST(sample_test_case, sample_test)
{
    EXPECT_EQ(1, 1);
}

int main(int argc, char **argv)
{
    // You should change here to set up your own URDF file or just pass it as an argument of this example.
    const std::string urdf_filename = PINOCCHIO_MODEL_DIR + std::string("/se3_object.urdf");

    pinocchio::Model model;
    pinocchio::urdf::buildModel(urdf_filename, model);

    mars::SE3 mars_model(model);
    mars_model.init();
    mars::SE3 mars_model2(urdf_filename);
    mars_model2.init();

    Eigen::VectorXd q = Eigen::VectorXd::Random(7);
    q.tail(4) = Eigen::Quaterniond::UnitRandom().coeffs();
    Eigen::VectorXd v = Eigen::VectorXd::Random(6);
    Eigen::VectorXd dv = Eigen::VectorXd::Random(6);

    Eigen::MatrixXd M = Eigen::MatrixXd::Zero(6, 6);
    M = mars_model.mass_matrix(q, M);
    std::cout << "M = \n" << M << std::endl;

    Eigen::VectorXd g = Eigen::VectorXd::Zero(6);
    g = mars_model.gravity_term(q, g);
    std::cout << "g = " << g.transpose() << std::endl;

    Eigen::VectorXd nle = Eigen::VectorXd::Zero(6);
    nle = mars_model.non_linear_term(q, v, nle);
    std::cout << "nle = " << nle.transpose() << std::endl;

    Eigen::VectorXd qdot = Eigen::VectorXd::Zero(7);
    qdot = mars_model.kinematic_ode(q, v, qdot);
    std::cout << "qdot = " << qdot.transpose() << std::endl;

    Eigen::MatrixXd J_0 = Eigen::MatrixXd::Zero(6, 6);
    J_0 = mars_model.frame_jacobian(0, q, J_0);
    std::cout << "J_0 = \n" << J_0 << std::endl;

    Eigen::MatrixXd J_1 = Eigen::MatrixXd::Zero(6, 6);
    J_1 = mars_model.frame_jacobian(1, q, J_1);
    std::cout << "J_1 = \n" << J_1 << std::endl;

    Eigen::VectorXd a_0 = Eigen::VectorXd::Zero(6);
    a_0 = mars_model.frame_local_acc(0, q, v, a_0);
    std::cout << "a_0 = " << a_0.transpose() << std::endl;

    Eigen::VectorXd a_1 = Eigen::VectorXd::Zero(6);
    a_1 = mars_model.frame_local_acc(1, q, v, a_1);
    std::cout << "a_1 = " << a_1.transpose() << std::endl;


    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}