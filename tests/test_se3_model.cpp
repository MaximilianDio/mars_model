#include <iostream>

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


    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}