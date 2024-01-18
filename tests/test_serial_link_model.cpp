#include <iostream>

#include "eigen3/Eigen/Dense"
#include "mars_model/serial_link_model.hpp"

#include "gtest/gtest.h"

// PINOCCHIO_MODEL_DIR is defined by the CMake but you can define your own directory here.
#ifndef PINOCCHIO_MODEL_DIR
#define PINOCCHIO_MODEL_DIR "/workspace/models/iiwa7"
#endif


TEST(TestConstructor, TestConstructorViaModel)
{
    // You should change here to set up your own URDF file or just pass it as an argument of this example.
    const std::string urdf_filename = PINOCCHIO_MODEL_DIR + std::string("/iiwa7.urdf");

    pinocchio::Model model;
    pinocchio::urdf::buildModel(urdf_filename, model);

    mars::SerialLink mars_model(model);
}

TEST(TestConstructor, TestConstructorViaURDF)
{
    // You should change here to set up your own URDF file or just pass it as an argument of this example.
    const std::string urdf_filename = PINOCCHIO_MODEL_DIR + std::string("/iiwa7.urdf");

    mars::SerialLink mars_model(urdf_filename);
}

class TestSerialLinkModel : public ::testing::Test
{
protected:
    virtual void SetUp()
    {
        // You should change here to set up your own URDF file or just pass it as an argument of this example.
        const std::string urdf_filename = PINOCCHIO_MODEL_DIR + std::string("/iiwa7.urdf");
        mars_model_ = std::make_shared<mars::SerialLink>(urdf_filename);

        nq_ = mars_model_->model_.nq;
        nv_ = mars_model_->model_.nv;

        // init state
        q_ = Eigen::VectorXd::Random(nq_);
        v_ = Eigen::VectorXd::Random(nv_);
        dv_ = Eigen::VectorXd::Random(nv_);
    }

    virtual void TearDown()
    {
    }

    std::shared_ptr<mars::SerialLink> mars_model_;

    Eigen::VectorXd q_;
    Eigen::VectorXd v_;
    Eigen::VectorXd dv_;

    double nq_;
    double nv_;
};

TEST_F(TestSerialLinkModel, TestInit)
{
    EXPECT_THROW(mars_model_->init({"wrong_link"}), std::invalid_argument);
    EXPECT_NO_THROW(mars_model_->init({"iiwa_link_ee"})) << "init failed";
}

TEST_F(TestSerialLinkModel, TestMassMatrix)
{
    mars_model_->init({"iiwa_link_ee"});
    Eigen::MatrixXd M = Eigen::MatrixXd::Zero(nv_, nv_);
    M = mars_model_->mass_matrix(q_, M);
    std::cout << "M = \n" << M << std::endl;

    EXPECT_TRUE(M.isApprox(M.transpose())) << "M is not symmetric";
}

TEST_F(TestSerialLinkModel, TestGravityTerm)
{
    mars_model_->init({"iiwa_link_ee"});
    Eigen::VectorXd g = Eigen::VectorXd::Zero(nv_);
    g = mars_model_->gravity_term(q_, g);
    std::cout << "g = " << g.transpose() << std::endl;
}

TEST_F(TestSerialLinkModel, TestNonLinearTerm)
{
    mars_model_->init({"iiwa_link_ee"});
    Eigen::VectorXd nle = Eigen::VectorXd::Zero(nv_);
    nle = mars_model_->non_linear_term(q_, v_, nle);
    std::cout << "nle = " << nle.transpose() << std::endl;
}

TEST_F(TestSerialLinkModel, TestKinematics)
{
    mars_model_->init({"iiwa_link_ee"});
    mars_model_->update_kinematics(q_, v_);

    pinocchio::SE3 Mi;
    Eigen::VectorXd xi = Eigen::VectorXd::Zero(6);
    Eigen::MatrixXd Ji = Eigen::MatrixXd::Zero(6, nv_);
    Eigen::VectorXd dxi = Eigen::VectorXd::Zero(6);
    int i = 0;
    Mi = mars_model_->frame_placement(i, Mi);
    std::cout << "M_" << i << " = \n" << Mi.toHomogeneousMatrix() << std::endl;

    xi = mars_model_->frame_velocity(i, xi);
    std::cout << "v_" << i << " = " << xi.transpose() << std::endl;

    Ji = mars_model_->frame_jacobian(i, Ji);
    std::cout << "J_" << i << " = \n" << Ji << std::endl;

    EXPECT_TRUE((Ji * v_).isApprox(xi)) << "J_" << i << " * v_ != v_" << i;

    dxi = mars_model_->frame_local_acc(i, dxi);
    std::cout << "local_a_" << i << " = " << dxi.transpose() << std::endl;
}


int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
// int main(int argc, char **argv)
// {
//     // You should change here to set up your own URDF file or just pass it as an argument of this example.
//     const std::string urdf_filename = PINOCCHIO_MODEL_DIR + std::string("/iiwa7.urdf");

//     pinocchio::Model model;
//     pinocchio::urdf::buildModel(urdf_filename, model);

//     mars::SerialLink mars_model(model);
//     mars_model.init({"iiwa_link_ee"});

//     Eigen::VectorXd q = Eigen::VectorXd::Random(mars_model.model_.nq);
//     Eigen::VectorXd v = Eigen::VectorXd::Random(mars_model.model_.nv);
//     Eigen::VectorXd dv = Eigen::VectorXd::Random(mars_model.model_.nv);

//     std::cout << "q = " << q.transpose() << std::endl;
//     std::cout << "v = " << v.transpose() << std::endl;
//     std::cout << "dv = " << dv.transpose() << std::endl;

//     Eigen::MatrixXd M = Eigen::MatrixXd::Zero(mars_model.model_.nv, mars_model.model_.nv);
//     M = mars_model.mass_matrix(q, M);
//     std::cout << "M = \n" << M << std::endl;

//     Eigen::VectorXd g = Eigen::VectorXd::Zero(mars_model.model_.nv);
//     g = mars_model.gravity_term(q, g);
//     std::cout << "g = " << g.transpose() << std::endl;

//     Eigen::VectorXd nle = Eigen::VectorXd::Zero(mars_model.model_.nv);
//     nle = mars_model.non_linear_term(q, v, nle);
//     std::cout << "nle = " << nle.transpose() << std::endl;

//     Eigen::VectorXd qdot = Eigen::VectorXd::Zero(mars_model.model_.nv);
//     qdot = mars_model.kinematic_ode(q, v, qdot);
//     std::cout << "qdot = " << qdot.transpose() << std::endl;

//     // required for all following kinematics functions
//     mars_model.update_kinematics(q, v);

//     pinocchio::SE3 frame_pose;
//     frame_pose = mars_model.frame_placement(0, frame_pose);
//     std::cout << "frame_pose = \n" << frame_pose.toHomogeneousMatrix() << std::endl;

//     Eigen::VectorXd x = Eigen::VectorXd::Zero(6);
//     x = mars_model.frame_velocity(0, x);
//     std::cout << "x = " << x.transpose() << std::endl;

//     Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6, mars_model.model_.nv);
//     J = mars_model.frame_jacobian(0, J);
//     std::cout << "J = \n" << J << std::endl;

//     Eigen::VectorXd a = Eigen::VectorXd::Zero(6);
//     a = mars_model.frame_local_acc(0, a);
//     std::cout << "a = " << a.transpose() << std::endl;


//     ::testing::InitGoogleTest(&argc, argv);
//     return RUN_ALL_TESTS();
// }