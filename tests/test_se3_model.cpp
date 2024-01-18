#include <iostream>

#include "eigen3/Eigen/Dense"
#include "mars_model/se3_model.hpp"

#include "gtest/gtest.h"

// PINOCCHIO_MODEL_DIR is defined by the CMake but you can define your own directory here.
#ifndef PINOCCHIO_MODEL_DIR
#define PINOCCHIO_MODEL_DIR "/workspace/models/object_3d"
#endif

TEST(TestConstructor, TestConstructorViaModel)
{
    // You should change here to set up your own URDF file or just pass it as an argument of this example.
    const std::string urdf_filename = PINOCCHIO_MODEL_DIR + std::string("/se3_object.urdf");

    pinocchio::Model model;
    pinocchio::urdf::buildModel(urdf_filename, model);

    mars::SE3 mars_model(model);
}

TEST(TestConstructor, TestConstructorViaURDF)
{
    // You should change here to set up your own URDF file or just pass it as an argument of this example.
    const std::string urdf_filename = PINOCCHIO_MODEL_DIR + std::string("/se3_object.urdf");

    mars::SE3 mars_model(urdf_filename);
}

class TestSE3Model : public ::testing::Test
{
protected:
    virtual void SetUp()
    {
        // You should change here to set up your own URDF file or just pass it as an argument of this example.
        const std::string urdf_filename = PINOCCHIO_MODEL_DIR + std::string("/se3_object.urdf");
        mars_model_ = std::make_shared<mars::SE3>(urdf_filename);

        nq_ = 7;
        nv_ = 6;
        // init state
        q_ = Eigen::VectorXd::Random(nq_);
        q_.tail(4) = Eigen::Quaterniond::UnitRandom().coeffs();
        v_ = Eigen::VectorXd::Random(nv_);
        dv_ = Eigen::VectorXd::Random(nv_);
    }

    virtual void TearDown()
    {
    }

    std::shared_ptr<mars::SE3> mars_model_;

    Eigen::VectorXd q_;
    Eigen::VectorXd v_;
    Eigen::VectorXd dv_;

    double nq_;
    double nv_;
};

TEST_F(TestSE3Model, TestInit)
{
    EXPECT_THROW(mars_model_->init({"obj_p_01", "obj_p_03"}), std::invalid_argument);
    EXPECT_NO_THROW(mars_model_->init({"obj_p_01", "obj_p_02"})) << "init failed";
}

TEST_F(TestSE3Model, TestMassMatrix)
{
    mars_model_->init({"obj_p_01", "obj_p_02"});
    Eigen::MatrixXd M = Eigen::MatrixXd::Zero(nv_, nv_);
    M = mars_model_->mass_matrix(q_, M);
    std::cout << "M = \n" << M << std::endl;

    EXPECT_TRUE(M.isApprox(M.transpose())) << "M is not symmetric";
}

TEST_F(TestSE3Model, TestGravityTerm)
{
    mars_model_->init({"obj_p_01", "obj_p_02"});
    Eigen::VectorXd g = Eigen::VectorXd::Zero(nv_);
    g = mars_model_->gravity_term(q_, g);
    std::cout << "g = " << g.transpose() << std::endl;
}

TEST_F(TestSE3Model, TestNonLinearTerm)
{
    mars_model_->init({"obj_p_01", "obj_p_02"});
    Eigen::VectorXd nle = Eigen::VectorXd::Zero(nv_);
    nle = mars_model_->non_linear_term(q_, v_, nle);
    std::cout << "nle = " << nle.transpose() << std::endl;
}

TEST_F(TestSE3Model, TestKinematics)
{
    mars_model_->init({"obj_p_01", "obj_p_02"});
    mars_model_->update_kinematics(q_, v_);

    pinocchio::SE3 Mi;
    Eigen::VectorXd vi = Eigen::VectorXd::Zero(6);
    Eigen::MatrixXd Ji = Eigen::MatrixXd::Zero(nv_, nv_);
    Eigen::VectorXd ai = Eigen::VectorXd::Zero(6);
    for (int i = 0; i < 2; i++)
    {
        Mi = mars_model_->frame_placement(i, Mi);
        std::cout << "M_" << i << " = \n" << Mi.toHomogeneousMatrix() << std::endl;

        vi = mars_model_->frame_velocity(i, vi);
        std::cout << "v_" << i << " = " << vi.transpose() << std::endl;

        Ji = mars_model_->frame_jacobian(i, Ji);
        std::cout << "J_" << i << " = \n" << Ji << std::endl;

        EXPECT_TRUE((Ji * v_).isApprox(vi)) << "J_" << i << " * v_ != v_" << i;

        ai = mars_model_->frame_local_acc(i, ai);
        std::cout << "local_a_" << i << " = " << ai.transpose() << std::endl;
    }
}


int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}